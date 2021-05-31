import math
import random
import libpyDirtMP as prx


prx.init_random(random.randint(1,999999))
acrobot = prx.two_link_acrobot("acrobot")
simulation_step = 0.01
prx.set_simulation_step(simulation_step)
print("Using simulation_step:", simulation_step)

start_state = [0, 0, 0, 0]
goal_state = [math.pi, 0, 0, 0]


obs_pose_1 = prx.transform()
obs_pose_2 = prx.transform()
obs_pose_3 = prx.transform()
obs_pose_4 = prx.transform()
obs_pose_1.setIdentity()
obs_pose_2.setIdentity()
obs_pose_3.setIdentity()
obs_pose_4.setIdentity()
obs_pose_1.translation(prx.vector( 20, 20,0.5))
obs_pose_2.translation(prx.vector(-20, 20,0.5))
obs_pose_3.translation(prx.vector( 20,-20,0.5))
obs_pose_4.translation(prx.vector(-20,-20,0.5))
b1 = prx.box.create_obstacle("b1", 1., 1., 1., obs_pose_1)
b2 = prx.box.create_obstacle("b2", 1., 1., 1., obs_pose_2)
b3 = prx.box.create_obstacle("b3", 1., 1., 1., obs_pose_3)
b4 = prx.box.create_obstacle("b4", 1., 1., 1., obs_pose_4)

obstacles = [b1, b2, b3, b4]
obs_names = ["b1", "b2", "b3", "b4"]
### To have an obstacle-free environment, uncomment the following lines (and comment the above)
# obstacles = []
# obs_names = []
wm = prx.world_model([acrobot], obstacles)
wm.create_context("context", ["acrobot"], obs_names)
context = wm.get_context("context");

planner = prx.sst("sst");
planner_spec = prx.sst_specification(context.system_group,context.collision_group);
planner_spec.delta_near = 0.9
planner_spec.delta_drain = 0.2


def acrobot_distance_function(s1, s2):
	cost = 0	
	s1a0 = s1[0] + prx.PRX_PI
	s1a1 = s1[1] + prx.PRX_PI
	s1a2 = s1[2] 
	s1a3 = s1[3] 
		
	s2a0 = s2[0] + prx.PRX_PI
	s2a1 = s2[1] + prx.PRX_PI
	s2a2 = s2[2] 
	s2a3 = s2[3] 

	a0 = min((2 * prx.PRX_PI) - abs(s1a0 - s2a0), abs(s1a0 - s2a0));
	a1 = min((2 * prx.PRX_PI) - abs(s1a1 - s2a1), abs(s1a1 - s2a1));
	a2 = s1a2 - s2a2;
	a3 = s1a3 - s2a3;

	cost =  a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3

	return math.sqrt(cost);

planner_spec.distance_function = prx.distance_function.set_df(acrobot_distance_function);

planner_spec.min_control_steps = 1
planner_spec.max_control_steps = 50

# planner_spec.random_seed = random.randint(1,999999);
planner_spec.bnb = True;

planner_query = prx.sst_query(context.system_group.get_state_space(),context.system_group.get_control_space());
planner_query.start_state = context.system_group.get_state_space().make_point()
planner_query.goal_state = context.system_group.get_state_space().make_point()
context.system_group.get_state_space().copy_point_from_vector(planner_query.start_state, start_state);
context.system_group.get_state_space().copy_point_from_vector(planner_query.goal_state, goal_state);

print("Start State:", planner_query.start_state)
print("Goal  State:", planner_query.goal_state)

planner_query.goal_region_radius = 0.5;
planner_query.get_visualization = True;

planner.link_and_setup_spec(planner_spec)
planner.preprocess()
planner.link_and_setup_query(planner_query)

### Note: Python slows down computation ==> more time might be needed
# checker = prx.condition_check("time", 60)
checker = prx.condition_check("iterations", 50000)

print("Resolving query...")
planner.resolve_query(checker)
planner.fulfill_query();


### This part is only to visualize the solution
if (planner_query.get_visualization):
	vis_group = prx.three_js_group([acrobot], obstacles)
	if ( len(planner_query.solution_traj) != 0 ) :
		vis_group.add_vis_infos(prx.info_geometry.FULL_LINE, planner_query.solution_traj, "acrobot/ball", context.system_group.get_state_space(), "0x000000");

	timestamp = 0.0
	for state in planner_query.solution_traj :
		context.system_group.get_state_space().copy_from_point(state);
		vis_group.snapshot_state(timestamp)
		timestamp += simulation_step

	vis_group.output_html("py_output.html");
