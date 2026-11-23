% Requires ROS Toolbox and Simulink. Creates read-only monitoring model.
setenv('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp');setenv('ROS_LOCALHOST_ONLY','0');
if isempty(getenv('ROS_DOMAIN_ID')),setenv('ROS_DOMAIN_ID','0');end
load_system('ros2lib');
b=find_system('ros2lib','LookUnderMasks','all','FollowLinks','on','Name','Subscribe');
assert(~isempty(b),'ROS 2 Subscribe block is unavailable.');
m='CC_robot_monitor';if bdIsLoaded(m),close_system(m,0);end
new_system(m);set_param(m,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','0.02','StopTime','inf');
add_block(b{1},[m '/Odometry'],'Position',[30 40 180 100]);
configureSub([m '/Odometry'],'/odometry/filtered','nav_msgs/Odometry');
add_block(b{1},[m '/Map pose'],'Position',[30 210 180 270]);
configureSub([m '/Map pose'],'/robot_pose','geometry_msgs/PoseStamped');
for item={'Odometry','Map pose'}
 name=item{1};add_block('simulink/Sinks/Terminator',[m '/' name ' new']);add_line(m,[name '/1'],[name ' new/1']);
end
add_block('simulink/Signal Routing/Bus Selector',[m '/Velocity'],'OutputSignals','twist.twist.linear.x,twist.twist.angular.z','Position',[240 50 290 110]);
add_line(m,'Odometry/2','Velocity/1');
add_block('simulink/Sinks/Scope',[m '/Velocity scope'],'NumInputPorts','2','Position',[400 50 460 110]);
add_line(m,'Velocity/1','Velocity scope/1');add_line(m,'Velocity/2','Velocity scope/2');
add_block('simulink/Signal Routing/Bus Selector',[m '/Position'],'OutputSignals','pose.position.x,pose.position.y','Position',[240 220 290 280]);
add_line(m,'Map pose/2','Position/1');
add_block('simulink/Sinks/XY Graph',[m '/Trajectory'],'Position',[400 220 460 280]);
add_line(m,'Position/1','Trajectory/1');add_line(m,'Position/2','Trajectory/2');
save_system(m,fullfile(tempdir,[m '.slx']));open_system(m);
fprintf('Verify ROS Network domain %s and CycloneDDS before clicking Run.\n',getenv('ROS_DOMAIN_ID'));
function configureSub(block,topic,type)
 names=fieldnames(get_param(block,'DialogParameters'));
 setMask(block,names,'topicSource','Specify your own',false);
 setMask(block,names,'topic',topic,true);setMask(block,names,'messageType',type,true);
 setMask(block,names,'sampleTime','0.02',false);
end
function setMask(block,names,key,value,required)
 idx=find(strcmpi(names,key),1);
 if ~isempty(idx),set_param(block,names{idx},value);
 elseif required,error('Missing parameter %s: check this ROS Toolbox release.',key);end
end
