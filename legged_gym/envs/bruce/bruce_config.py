from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class BRUCECfg( LeggedRobotCfg ):
    class env(LeggedRobotCfg.env):
        num_envs=10
        num_actions=16
        num_observations=60
        env_spacing = 6

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane'
        measure_heights = False

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.5] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'hip_yaw_l' :0.,   # [rad]
            'hip_roll_l':0.,   # [rad]
            'hip_pitch_l' :0.,   # [rad]
            'knee_pitch_l' :0.,   # [rad]
            'ankle_pitch_l':0.,   # [rad]
            'hip_yaw_r' :0.,   # [rad]
            'hip_roll_r' :0.,   # [rad]
            'hip_pitch_r' :0.,   # [rad]
            'knee_pitch_r' :0.,   # [rad]
            'ankle_pitch_r' :0.,   # [rad]
            'shoulder_pitch_l' :0.,   # [rad]
            'shoulder_roll_l' :0.,   # [rad]
            'elbow_pitch_l' :0.,   # [rad]
            'shoulder_pitch_r' :0.,   # [rad]
            'shoulder_roll_r' :0.,   # [rad]
            'elbow_pitch_r' :0.,   # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        #stiffness:kp
        # stiffness = {'_':1}
        stiffness = {'hip_yaw':1,'hip_roll':1,'hip_pitch':1,'knee_pitch':1,'ankle_pitch':1,'shoulder_pitch':0.3,'shoulder_roll':0.3,'elbow_pitch':0.3}
        #damping:kd
        # damping={'_':0.001}
        damping={'hip_yaw':0.003,'hip_roll':0.003,'hip_pitch':0.003,'knee_pitch':0.003,'ankle_pitch':0.003,'shoulder_pitch':0.003,'shoulder_roll':0.003,'elbow_pitch':0.003}
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        use_actuator_network = True
        actuator_net_file = "{LEGGED_GYM_ROOT_DIR}/resources/actuator_nets/anydrive_v3_lstm.pt"

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/bruce/urdf/bruce.urdf'
        name = "bruce"
        foot_name = "foot"
        penalize_contacts_on = []
        terminate_after_contacts_on = ["base_link"]
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False

        # fix_base_link=True
        # armature=0.01
        angular_damping = 0.1
        linear_damping = 0.1
  
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9  #实际关节限制角度=最大关节限制角度*该比例
        base_height_target = 0.5
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0002
            dof_pos_limits = -10.0
        
        # use_terminal_body_height = True

    class viewer(LeggedRobotCfg.viewer):
        pos = [0, 0, 3]  # [m]
        lookat = [5, 5, 2]  # [m]

class BRUCECfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'bruce'
        load_run = -1
        max_iterations = 30000

  