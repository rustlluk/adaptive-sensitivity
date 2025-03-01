import PyKDL as kdl
import numpy as np
import rospy


class KDLHandle:
    def __init__(self, end_effector, kdl_tree, base_link):
        self.end_effector = end_effector
        # Create chain from base_link to end-effector
        self.chain = kdl_tree.getChain(base_link, self.end_effector)
        # Init solver for the given chain;
        self.vel_solver = kdl.ChainIkSolverVel_pinv(self.chain)
        # Get num active joints (active = not fixed)
        self.num_joints = self.chain.getNrOfJoints()
        # Init output array for joint velocities
        self.velocities = kdl.JntArray(self.num_joints)
        # FK solver
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        # output frame for FK
        self.fk_frame = kdl.Frame()
        # Jac solver
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        # jac output
        self.jac = kdl.Jacobian(self.num_joints)
        # dyn solver
        self.dyn_solver = kdl.ChainDynParam(self.chain, kdl.Vector(0, 0, -9.81))
        # dyn output
        self.dyn = kdl.JntSpaceInertiaMatrix(self.num_joints)
        # FK vel solver
        self.fk_vel_solver = kdl.ChainFkSolverVel_recursive(self.chain)
        # output frame for FK vel
        self.fk_vel_frame = kdl.FrameVel()

        segment_mass = 0
        segments = self.chain.getNrOfSegments()
        for s in range(segments):
            segment_mass += self.chain.getSegment(s).getInertia().getMass()

        self.mass = segment_mass

    @staticmethod
    def np_to_kdl(ar):
        kdl_ar = kdl.JntArray(len(ar))
        for idx, _ in enumerate(ar):
            kdl_ar[idx] = _
        return kdl_ar

    @staticmethod
    def kdl_to_np(ar, rows=0):
        if rows == 0:
            rows = ar.rows()
        python_ar = np.zeros(rows)
        for idx, _ in enumerate(ar):
            python_ar[idx] = _
        return python_ar

    @staticmethod
    def kdl_to_np_mat(mat, rows=0, cols=0):
        if rows == 0:
            rows = mat.rows()
        if cols == 0:
            cols = mat.columns()
        out = np.zeros((rows, cols))
        for i in range(rows):
            for j in range(cols):
                out[i, j] = mat[i, j]
        return out


class AirskinFeedback:
    PADS = [i for i in range(0, 11)]  # 10 pads

    # SIM Threshold values
    THRESHOLD_LOW = [15] * len(PADS)  # Uniform most sensitive threshold
    THRESHOLD_MED = [100] * len(PADS)  # Uniform medium sensitive threshold
    THRESHOLD_HIGH = [250] * len(PADS)  # Uniform least sensitive threshold

    def __init__(self, setup, lag):
        self.setup = setup  # SIM or REAL not used in current version
        if self.setup == "real":
            from airskin.msg import AirskinStatus, AirskinTouch
        else:
            from bullet_ros_ur.msg import AirskinStatus, AirskinTouch
        pressure = []
        for _ in range(2 * lag):
            pressure.append(rospy.wait_for_message("/airskin_status", AirskinStatus).pressures)

        self.delay = lag
        self.lag = lag
        self.prev_touched = np.array([False] * len(self.PADS))
        # self.touch = [0] * len(self.PADS)
        self.signals = [False] * np.shape(pressure)[1]
        self.calc_signals = [False] * np.shape(pressure)[1]
        self.y = np.array(pressure)
        self.filteredY = np.copy(self.y[-self.lag:])
        self.filteredStdY = np.copy(self.y[-self.lag - self.delay: - self.lag])

        assert len(self.filteredY) == self.lag
        assert len(self.filteredStdY) == self.lag

        self.avgFilter = np.mean(self.filteredY, axis=0)
        self.stdFilter = np.std(self.filteredStdY, axis=0)
        self.index = 0
        self.touch_level = np.array([0] * len(self.PADS))

        self.mode = 0

        self.pub = rospy.Publisher("/airskin_pain/touch", AirskinTouch, queue_size=1)
        self.msg = AirskinTouch()

        rospy.logerr("Initialization completed")

    def __call__(self, msg):
        if self.index >= self.lag:
            self.index = 0

        new_value = msg.pressures
        new_peak = np.subtract(new_value, self.avgFilter)

        low_touch = np.multiply(self.THRESHOLD_LOW, self.stdFilter)
        med_touch = np.multiply(self.THRESHOLD_MED, self.stdFilter)
        high_touch = np.multiply(self.THRESHOLD_HIGH, self.stdFilter)

        self.signals = new_peak > low_touch  # Detect airskin touch
        self.touch_level[self.signals == True] = 1
        if np.any(self.signals):
            for i, j in enumerate([med_touch, high_touch]):
                level = new_peak > j
                self.touch_level[level == True] = i + 2

        self.prev_touched = self.signals

        touch = np.where(self.signals)[0]
        touch_level = self.touch_level[touch]
        # rospy.logwarn(f'Touch {touch}; Level {touch_level}')  # Debug print
        self.msg.touch = list(touch)
        self.msg.level = list(touch_level)
        self.pub.publish(self.msg)

        self.calc_signals = np.add(self.signals, (-new_peak > low_touch))

        calc_touch = np.multiply(self.filteredY[self.index],
                                 self.calc_signals)  # original self.filteredY[i-1] instead of self.avgFilter
        calc_no_touch = np.multiply(new_value, np.logical_not(self.calc_signals))

        calc_no_touch_delay = np.multiply(self.filteredY[self.index], np.logical_not(self.calc_signals))
        calc_touch_delay = np.multiply(self.filteredStdY[self.index], self.calc_signals)

        self.filteredY[self.index] = np.add(calc_touch, calc_no_touch)
        self.filteredStdY[self.index] = np.add(calc_touch_delay, calc_no_touch_delay)
        self.avgFilter = np.mean(self.filteredY, axis=0)

        newStdFilter = np.std(self.filteredStdY, axis=0)
        self.stdFilter[np.logical_not(self.calc_signals)] = newStdFilter[np.logical_not(self.calc_signals)]

        self.index += 1
