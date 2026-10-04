#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <mutex>
#include <numeric>
#include <sstream>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_msgs/msg/tf_message.hpp>

namespace mtc = moveit::task_constructor;

const std::string ARM = "arm";
const std::string HAND = "gripper";
const std::string HAND_FRAME = "tool_link";

// Scene layout (base_link frame, metres). Tune these if the task fails to plan.
const double TABLE_TOP = 0.15;
const double OBJECT_SIZE[3] = {0.04, 0.04, 0.04};
// Cubes float this far above the table in the planning scene: touching counts as colliding once attached
const double CUBE_LIFT = 0.001;
// Cube spot used when no cube_poses are given (x, y, yaw); pick_place.launch.py randomises them
const std::vector<double> DEFAULT_CUBE_POSES = {0.6, -0.2, 0.0};
// Shallow tray with 2x3 slots, centred on the right half of the table (matches table.sdf).
// 9 cm pitch leaves 2 cm between a half-open finger and the next cube; walls stay below the
// finger tips (1.5 cm above the cube bottom) so the fingers pass over them.
const double TRAY_XY[2] = {0.6, 0.2};
const double TRAY_INNER[2] = {0.18, 0.27};
const double TRAY_WALL = 0.01;     // thickness
const double TRAY_FLOOR = 0.005;   // thickness
const double TRAY_HEIGHT = 0.015;  // floor bottom to wall top
const double SLOT_PITCH = 0.09;
const int SLOT_ROWS = 2;  // along x
const int SLOT_COLS = 3;  // along y
// Release this far above the tray floor so the box settles instead of being pushed into it
const double RELEASE_GAP = 0.003;
// Object centre along tool_link z when grasped: fingers span 0.02-0.10 m past tool_link,
// so the tips stop 1.5 cm above the cube bottom (clear of the table and the 1 cm tray walls)
const double GRASP_DEPTH = 0.095;
// SRDF gripper state aimed 1 cm inside the cube; in Gazebo the force-controlled fingers stop on it
const std::string GRASP_STATE = "gripper_grasp";
// Give up on a cube after this many tries (planning or execution), so a stuck cube can't loop forever
const int MAX_ATTEMPTS = 3;
// Gazebo: each cube publishes its real pose here (spawn_cubes.py, bridged in arm_gz.launch.xml)
std::string gzPoseTopic(int i) { return "/model/cube_" + std::to_string(i) + "/pose"; }

moveit_msgs::msg::CollisionObject makeBox(const std::string &id, double sx, double sy, double sz,
                                          double x, double y, double z, double yaw = 0.0)
{
    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = "base_link";
    obj.primitives.resize(1);
    obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    obj.primitives[0].dimensions = {sx, sy, sz};
    obj.pose.position.x = x;
    obj.pose.position.y = y;
    obj.pose.position.z = z;
    obj.pose.orientation.z = std::sin(yaw / 2);
    obj.pose.orientation.w = std::cos(yaw / 2);
    obj.operation = obj.ADD;
    return obj;
}

// Floor plus 4 walls, all as primitives of one object
moveit_msgs::msg::CollisionObject makeTray()
{
    const double ox = TRAY_INNER[0] + 2 * TRAY_WALL, oy = TRAY_INNER[1] + 2 * TRAY_WALL;
    const double wall_x = (TRAY_INNER[0] + TRAY_WALL) / 2, wall_y = (TRAY_INNER[1] + TRAY_WALL) / 2;
    const double parts[5][6] = {  // size xyz, centre xyz (relative to tray centre on the table)
        {ox, oy, TRAY_FLOOR, 0, 0, TRAY_FLOOR / 2},
        {TRAY_WALL, oy, TRAY_HEIGHT, -wall_x, 0, TRAY_HEIGHT / 2},
        {TRAY_WALL, oy, TRAY_HEIGHT, wall_x, 0, TRAY_HEIGHT / 2},
        {ox, TRAY_WALL, TRAY_HEIGHT, 0, -wall_y, TRAY_HEIGHT / 2},
        {ox, TRAY_WALL, TRAY_HEIGHT, 0, wall_y, TRAY_HEIGHT / 2},
    };
    moveit_msgs::msg::CollisionObject tray;
    tray.id = "tray";
    tray.header.frame_id = "base_link";
    tray.pose.position.x = TRAY_XY[0];
    tray.pose.position.y = TRAY_XY[1];
    tray.pose.position.z = TABLE_TOP;
    tray.pose.orientation.w = 1.0;
    for (const auto &p : parts) {
        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {p[0], p[1], p[2]};
        geometry_msgs::msg::Pose pose;
        pose.position.x = p[3];
        pose.position.y = p[4];
        pose.position.z = p[5];
        pose.orientation.w = 1.0;
        tray.primitives.push_back(prim);
        tray.primitive_poses.push_back(pose);
    }
    tray.operation = tray.ADD;
    return tray;
}

// Slot centre in base_link, slot 0 is the -x, -y corner, counting along y first
std::array<double, 2> slotXY(int slot)
{
    const int row = slot / SLOT_COLS, col = slot % SLOT_COLS;
    return {TRAY_XY[0] + (row - (SLOT_ROWS - 1) / 2.0) * SLOT_PITCH,
            TRAY_XY[1] + (col - (SLOT_COLS - 1) / 2.0) * SLOT_PITCH};
}

struct Cube
{
    std::string id;
    geometry_msgs::msg::Pose pose;  // as measured
    bool upright;                   // a face points up, so it can be grasped from above
    double yaw;                     // of the faces around world z, when upright
};

// A cube is symmetric, so any face up is fine: find the cube axis closest to world z
Cube makeCube(const std::string &id, const geometry_msgs::msg::Pose &pose)
{
    const auto &o = pose.orientation;
    const Eigen::Matrix3d rot = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized().toRotationMatrix();
    int up = 0;
    for (int k = 1; k < 3; ++k) {
        if (std::abs(rot(2, k)) > std::abs(rot(2, up))) up = k;
    }
    const int side = (up + 1) % 3;
    return {id, pose, std::abs(rot(2, up)) > 0.97, std::atan2(rot(1, side), rot(0, side))};
}

bool inTray(const Cube &c)
{
    return std::abs(c.pose.position.x - TRAY_XY[0]) < TRAY_INNER[0] / 2 &&
           std::abs(c.pose.position.y - TRAY_XY[1]) < TRAY_INNER[1] / 2 &&
           c.pose.position.z < TABLE_TOP + 0.1;
}

// Upright and standing on the table top (not fallen off, not leaning on something)
bool onTable(const Cube &c)
{
    return c.upright && std::abs(c.pose.position.z - (TABLE_TOP + OBJECT_SIZE[2] / 2)) < 0.01 &&
           std::abs(c.pose.position.x - 0.65) < 0.2 && std::abs(c.pose.position.y) < 0.4;
}

int nearestSlot(const Cube &c)
{
    int best = 0;
    for (int i = 1; i < SLOT_ROWS * SLOT_COLS; ++i) {
        auto d = [&](int s) {
            const auto xy = slotXY(s);
            return std::hypot(c.pose.position.x - xy[0], c.pose.position.y - xy[1]);
        };
        if (d(i) < d(best)) best = i;
    }
    return best;
}

// Latest Gazebo cube poses by name, filled by the subscriptions in main()
struct GzPoses
{
    std::mutex mutex;
    std::map<std::string, geometry_msgs::msg::TransformStamped> latest;
    size_t count = 0;

    // Poses received after the call, so they show the world after the last motion
    bool waitFresh(std::map<std::string, geometry_msgs::msg::TransformStamped> &out)
    {
        size_t start;
        {
            std::lock_guard<std::mutex> lock(mutex);
            start = count;
        }
        for (int i = 0; i < 100; ++i) {  // 5 s
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::lock_guard<std::mutex> lock(mutex);
            if (count > start) break;
        }
        // Cubes publish at 20 Hz each: after one fresh message, wait for the others to catch up
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::lock_guard<std::mutex> lock(mutex);
        out = latest;
        return count > start;
    }
};

// Where the cubes are now. In Gazebo they are measured and the planning scene is moved to match;
// on mock hardware nothing moves by itself, so the planning scene already is the truth.
bool measureCubes(bool sim, GzPoses &gz, std::vector<Cube> &cubes)
{
    moveit::planning_interface::PlanningSceneInterface psi;
    // A task that failed mid-carry leaves its cube attached to the hand; put it back in the world
    for (const auto &[id, obj] : psi.getAttachedObjects()) {
        moveit_msgs::msg::AttachedCollisionObject detach;
        detach.object.id = id;
        detach.object.operation = detach.object.REMOVE;
        psi.applyAttachedCollisionObject(detach);
    }

    cubes.clear();
    if (!sim) {
        for (const auto &[id, pose] : psi.getObjectPoses(psi.getKnownObjectNames())) {
            if (id.rfind("cube_", 0) == 0) cubes.push_back(makeCube(id, pose));
        }
        return true;
    }

    std::map<std::string, geometry_msgs::msg::TransformStamped> measured;
    if (!gz.waitFresh(measured)) return false;
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    for (const auto &[id, t] : measured) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = t.transform.translation.x;
        pose.position.y = t.transform.translation.y;
        pose.position.z = t.transform.translation.z;
        pose.orientation = t.transform.rotation;
        const Cube cube = makeCube(id, pose);
        cubes.push_back(cube);

        auto obj = makeBox(cube.id, OBJECT_SIZE[0], OBJECT_SIZE[1], OBJECT_SIZE[2], pose.position.x,
                           pose.position.y, pose.position.z + CUBE_LIFT, cube.yaw);
        if (!cube.upright) {
            obj.pose.orientation = pose.orientation;  // tipped over: keep the real pose for collisions
        }
        if (pose.position.z < TABLE_TOP) {
            obj.operation = obj.REMOVE;  // fell off the table, out of reach
        }
        objects.push_back(obj);
    }
    psi.applyCollisionObjects(objects);
    return true;
}

void setupPlanningScene(const std::vector<double> &poses)
{
    moveit::planning_interface::PlanningSceneInterface psi;
    // Clear cubes from an earlier run, including one left attached to the hand by a stopped task
    for (const auto &[id, obj] : psi.getAttachedObjects()) {
        moveit_msgs::msg::AttachedCollisionObject detach;
        detach.object.id = id;
        detach.object.operation = detach.object.REMOVE;
        psi.applyAttachedCollisionObject(detach);
    }
    psi.removeCollisionObjects(psi.getKnownObjectNames());

    std::vector<moveit_msgs::msg::CollisionObject> objects = {
        makeBox("table", 0.4, 0.8, TABLE_TOP, 0.65, 0.0, TABLE_TOP / 2),
        makeTray(),
    };
    for (size_t i = 0; i < poses.size() / 3; ++i) {
        objects.push_back(makeBox("cube_" + std::to_string(i), OBJECT_SIZE[0], OBJECT_SIZE[1], OBJECT_SIZE[2],
                                  poses[3 * i], poses[3 * i + 1], TABLE_TOP + OBJECT_SIZE[2] / 2 + CUBE_LIFT,
                                  poses[3 * i + 2]));
    }
    psi.applyCollisionObjects(objects);
}

mtc::Task createTask(const rclcpp::Node::SharedPtr &node, const std::string &object, int slot)
{
    mtc::Task task;
    task.stages()->setName("pick and place " + object);
    task.loadRobotModel(node);
    task.setProperty("group", ARM);
    task.setProperty("eef", HAND);
    task.setProperty("ik_frame", HAND_FRAME);

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setStepSize(0.01);

    auto hand_links = task.getRobotModel()->getJointModelGroup(HAND)->getLinkModelNamesWithCollisionGeometry();

    geometry_msgs::msg::Vector3Stamped world_up;
    world_up.header.frame_id = "base_link";
    world_up.vector.z = 1.0;

    auto current = std::make_unique<mtc::stages::CurrentState>("current");
    mtc::Stage *current_ptr = current.get();
    task.add(std::move(current));

    auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    open_hand->setGroup(HAND);
    open_hand->setGoal("gripper_open");
    task.add(std::move(open_hand));

    auto move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{{ARM, sampling_planner}});
    move_to_pick->setTimeout(5.0);
    move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(move_to_pick));

    mtc::Stage *attach_ptr = nullptr;
    {
        auto pick = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(pick->properties(), {"eef", "group", "ik_frame"});
        pick->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        // Come straight down onto the object along the tool axis
        auto approach = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
        approach->properties().set("link", HAND_FRAME);
        approach->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        approach->setMinMaxDistance(0.08, 0.15);
        geometry_msgs::msg::Vector3Stamped tool_forward;
        tool_forward.header.frame_id = HAND_FRAME;
        tool_forward.vector.z = 1.0;
        approach->setDirection(tool_forward);
        pick->insert(std::move(approach));

        // Sample grasps around the object's vertical axis, tool pointing down.
        // 90 deg steps keep the fingers flat on the box faces; a diagonal grasp pinches the
        // corners (5.7 cm across) and squirts the box out in Gazebo.
        auto grasp = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        grasp->properties().configureInitFrom(mtc::Stage::PARENT);
        grasp->setPreGraspPose("gripper_open");
        grasp->setObject(object);
        grasp->setAngleDelta(M_PI / 2);
        grasp->setMonitoredStage(current_ptr);

        Eigen::Isometry3d grasp_frame = Eigen::Isometry3d::Identity();
        grasp_frame.translation().z() = GRASP_DEPTH;
        grasp_frame.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

        auto grasp_ik = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(grasp));
        grasp_ik->setMaxIKSolutions(8);
        grasp_ik->setMinSolutionDistance(1.0);
        grasp_ik->setIKFrame(grasp_frame, HAND_FRAME);
        grasp_ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        grasp_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        pick->insert(std::move(grasp_ik));

        auto allow = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
        allow->allowCollisions(object, hand_links, true);
        pick->insert(std::move(allow));

        auto close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        close_hand->setGroup(HAND);
        close_hand->setGoal(GRASP_STATE);
        pick->insert(std::move(close_hand));

        auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        attach->attachObject(object, HAND_FRAME);
        attach_ptr = attach.get();
        pick->insert(std::move(attach));

        auto lift = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
        lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        lift->setMinMaxDistance(0.05, 0.2);
        lift->setIKFrame(HAND_FRAME);
        lift->setDirection(world_up);
        pick->insert(std::move(lift));

        task.add(std::move(pick));
    }

    // Arm only: the hand stays closed, and no SRDF group spans arm + hand for a merged trajectory
    auto move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{{ARM, sampling_planner}});
    move_to_place->setTimeout(5.0);
    move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(move_to_place));

    {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
        place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

        // Come straight down into the slot so the box never swings over the tray walls
        auto lower = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
        lower->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        lower->setMinMaxDistance(0.03, 0.1);
        lower->setIKFrame(HAND_FRAME);
        geometry_msgs::msg::Vector3Stamped world_down = world_up;
        world_down.vector.z = -1.0;
        lower->setDirection(world_down);
        place->insert(std::move(lower));

        auto place_pose = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        place_pose->properties().configureInitFrom(mtc::Stage::PARENT);
        place_pose->setObject(object);
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        const auto xy = slotXY(slot);
        target.pose.position.x = xy[0];
        target.pose.position.y = xy[1];
        target.pose.position.z = TABLE_TOP + TRAY_FLOOR + OBJECT_SIZE[2] / 2 + RELEASE_GAP;
        target.pose.orientation.w = 1.0;
        place_pose->setPose(target);
        place_pose->setMonitoredStage(attach_ptr);

        auto place_ik = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(place_pose));
        place_ik->setMaxIKSolutions(4);
        place_ik->setMinSolutionDistance(1.0);
        place_ik->setIKFrame(object);
        place_ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        place_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        place->insert(std::move(place_ik));

        // Half open only: fully open fingers would reach into the neighbouring slots
        auto open = std::make_unique<mtc::stages::MoveTo>("release", interpolation_planner);
        open->setGroup(HAND);
        open->setGoal("gripper_half_open");
        place->insert(std::move(open));

        auto forbid = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
        forbid->allowCollisions(object, hand_links, false);
        place->insert(std::move(forbid));

        auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        detach->detachObject(object, HAND_FRAME);
        place->insert(std::move(detach));

        auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        retreat->setMinMaxDistance(0.05, 0.2);
        retreat->setIKFrame(HAND_FRAME);
        retreat->setDirection(world_up);
        place->insert(std::move(retreat));

        task.add(std::move(place));
    }

    // Wait above the table for the next cube (SRDF "ready"); main() sends the arm home once every cube is packed
    auto ready = std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
    ready->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    ready->setGoal("ready");
    task.add(std::move(ready));

    return task;
}

bool runTask(const rclcpp::Node::SharedPtr &node, mtc::Task &task)
{
    try {
        task.init();
    }
    catch (mtc::InitStageException &e) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Task init failed: " << e);
        return false;
    }
    if (!task.plan(5) || task.solutions().empty()) {
        std::ostringstream why;
        task.explainFailure(why);
        RCLCPP_ERROR(node->get_logger(), "Planning %s failed:\n%s", task.name().c_str(), why.str().c_str());
        return false;
    }
    task.introspection().publishSolution(*task.solutions().front());
    auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Executing %s failed with code %d", task.name().c_str(), result.val);
        return false;
    }
    return true;
}

mtc::Task createHomeTask(const rclcpp::Node::SharedPtr &node)
{
    mtc::Task task;
    task.stages()->setName("go home");
    task.loadRobotModel(node);
    task.setProperty("group", ARM);
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));
    auto home = std::make_unique<mtc::stages::MoveTo>("return home", std::make_shared<mtc::solvers::PipelinePlanner>(node));
    home->setGroup(ARM);
    home->setGoal("home");
    task.add(std::move(home));
    return task;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("pick_place", options);

    GzPoses gz;
    std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> gz_subs;
    for (int i = 0; i < SLOT_ROWS * SLOT_COLS; ++i) {
        gz_subs.push_back(node->create_subscription<tf2_msgs::msg::TFMessage>(
            gzPoseTopic(i), 10, [&gz](const tf2_msgs::msg::TFMessage &msg) {
                std::lock_guard<std::mutex> lock(gz.mutex);
                for (const auto &t : msg.transforms) {
                    if (t.child_frame_id.rfind("cube_", 0) == 0) gz.latest[t.child_frame_id] = t;
                }
                ++gz.count;
            }));
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    const bool sim = node->get_parameter_or("sim", false);
    const auto poses = node->get_parameter_or("cube_poses", DEFAULT_CUBE_POSES);
    if (poses.size() % 3 != 0 || poses.size() / 3 > SLOT_ROWS * SLOT_COLS) {
        RCLCPP_ERROR(node->get_logger(), "cube_poses must be x, y, yaw triples, at most %d cubes",
                     SLOT_ROWS * SLOT_COLS);
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }
    setupPlanningScene(poses);

    // Measure, pick the nearest cube that isn't in the tray, repeat. Re-measuring after every
    // cube catches ones that slipped, got knocked or missed their slot, and retries them.
    // Tasks stay alive so every solution remains browsable in RViz's Motion Planning Tasks panel
    std::vector<std::unique_ptr<mtc::Task>> tasks;
    std::map<std::string, int> attempts;
    std::vector<Cube> cubes;
    while (rclcpp::ok()) {
        if (!measureCubes(sim, gz, cubes) || cubes.size() != poses.size() / 3) {
            RCLCPP_ERROR(node->get_logger(), "Measured %zu of %zu cubes on /model/cube_N/pose, "
                         "is arm_gz.launch.xml running?", cubes.size(), poses.size() / 3);
            break;
        }
        std::vector<bool> occupied(SLOT_ROWS * SLOT_COLS, false);
        const Cube *next = nullptr;
        for (const auto &c : cubes) {
            if (inTray(c)) {
                occupied[nearestSlot(c)] = true;
            }
            else if (onTable(c) && attempts[c.id] < MAX_ATTEMPTS &&
                     (!next || std::hypot(c.pose.position.x, c.pose.position.y) <
                                   std::hypot(next->pose.position.x, next->pose.position.y))) {
                next = &c;
            }
        }
        const auto free_slot = std::find(occupied.begin(), occupied.end(), false);
        if (!next || free_slot == occupied.end()) break;
        const int slot = free_slot - occupied.begin();

        ++attempts[next->id];
        RCLCPP_INFO(node->get_logger(), "%s at (%.3f, %.3f) -> slot %d (try %d)", next->id.c_str(),
                    next->pose.position.x, next->pose.position.y, slot, attempts[next->id]);
        tasks.push_back(std::make_unique<mtc::Task>(createTask(node, next->id, slot)));
        runTask(node, *tasks.back());
        if (sim) {
            std::this_thread::sleep_for(std::chrono::seconds(1));  // let the released cube settle
        }
    }

    int packed = 0;
    for (const auto &c : cubes) {
        if (inTray(c)) {
            ++packed;
        }
        else {
            RCLCPP_WARN(node->get_logger(), "%s left out at (%.3f, %.3f, %.3f)%s", c.id.c_str(), c.pose.position.x,
                        c.pose.position.y, c.pose.position.z,
                        !onTable(c) ? ": not standing on the table" : ": gave up after retries");
        }
    }
    RCLCPP_INFO(node->get_logger(), "Packed %d/%zu cubes", packed, cubes.size());

    tasks.push_back(std::make_unique<mtc::Task>(createHomeTask(node)));
    if (runTask(node, *tasks.back())) {
        RCLCPP_INFO(node->get_logger(), "Job done, arm is home");
    }

    spinner.join();
    return 0;
}
