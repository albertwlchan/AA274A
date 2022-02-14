import numpy as np

class RRTStar(object):
    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, obstacles):
        self.statespace_lo = np.array(statespace_lo)    # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = np.array(statespace_hi)    # state space upper bound (e.g., [5, 5])
        self.x_init = np.array(x_init)                  # initial state
        self.x_goal = np.array(x_goal)                  # goal state
        self.obstacles = obstacles                      # obstacle set (line segments)
        self.path = None        # the final path as a list of states

    def is_free_motion(self, obstacles, x1, x2):
        raise NotImplementedError("is_free_motion must be overriden by a subclass of RRT")

    def find_nearest(self, V, x):
        raise NotImplementedError("find_nearest must be overriden by a subclass of RRT")

    def find_near(self, V, x):
        raise NotImplementedError("find_near must be overriden by a subclass of RRT")

    def steer_towards(self, x1, x2, eps):
        raise NotImplementedError("steer_towards must be overriden by a subclass of RRT")

    def solve(self, eps, max_iters=1000, goal_bias=0.05, shortcut=False):
        state_dim = len(self.x_init)

        V = np.zeros((max_iters + 1, state_dim))
        V[0,:] = self.x_init    # RRT is rooted at self.x_init
        cost = np.zeros((max_iters + 1,))
        cost[0] = 0
        n = 1                   # the current size of the RRT (states accessible as V[range(n),:])

        P = -np.ones(max_iters + 1, dtype=int)
        success = False

        ########## Code starts here ##########
        i = 1
        while i <= max_iters:
            z = np.random.random()
            if z < goal_bias:
                x_rand = self.x_goal
            else:
                x_rand = np.random.uniform(low=self.statespace_lo, 
                                           high=self.statespace_hi)
            j = self.find_nearest(V[:i, :], x_rand) # j: index of x_nearest
            x_nearest = V[j, :]
            x_new = self.steer_towards(x_nearest, x_rand, eps)
            if self.is_free_motion(self.obstacles, x_nearest, x_new):
                near_indices = self.find_near(V[:i, :], x_new, 2*eps) # indices of near vertices
                V[i, :] = x_new

                x_min = x_nearest
                c_min = cost[j] + self.get_cost(x_nearest, x_new)
                
                for k in near_indices:
                    x_near = V[k, :]
                    c_thru_near = cost[k] + self.get_cost(x_near, x_new)
                    if self.is_free_motion(self.obstacles, x_near, x_new) and c_thru_near < c_min:
                        x_min = x_near
                        j = k
                        c_min = c_thru_near
                P[i] = j
                cost[i] = c_min
                for k in near_indices:
                    x_near = V[k, :]
                    c_near_thru_new = cost[i] + self.get_cost(x_new, x_near)
                    if self.is_free_motion(self.obstacles, x_new, x_near) and c_thru_near < cost[k]:
                        P[k] = i

                if np.all(x_new == self.x_goal):
                    success = True
                    break
                i += 1
        
        if success:
            self.path = []
            while i >= 0:
                self.path.append(V[i]) # reversely append
                i = P[i]
            self.path.reverse()     
        ########## Code ends here ##########
        return success

    def shortcut_path(self):
        success = False
        while not success:
            success = True
            i = 1
            while i < len(self.path)-1:
                if self.is_free_motion(self.obstacles, 
                                       self.path[i-1], 
                                       self.path[i+1]):
                    del self.path[i]
                    success = False
                else:
                    i += 1 

class GeometricRRTStar(RRTStar):
    def get_cost(self, x1, x2):
        return np.linalg.norm(x2-x1)

    def find_nearest(self, V, x):
        return np.linalg.norm(V-x, axis=1).argmin()
    
    def find_near(self, V, x, radius):
        return np.where(np.linalg.norm(V-x, axis=1) < radius)[0]

    def steer_towards(self, x1, x2, eps):
        return x2 if np.linalg.norm(x2-x1) < eps else x1 + (eps/np.linalg.norm(x2-x1)) * (x2-x1)

    def is_free_motion(self, obstacles, x1, x2):
        # motion = np.array([x1, x2])
        # for line in obstacles:
        #     if line_line_intersection(motion, line):
        #         return False
        # return True
        npoints = int(np.ceil(2.0 * np.linalg.norm(np.array(x2) - np.array(x1)) / obstacles.resolution))
        points = [obstacles.snap_to_grid(np.array(x1) + alpha * (np.array(x2) - np.array(x1))) for alpha in np.linspace(0, 1, npoints)]
        return np.all([obstacles.is_free(point) for point in points]) 
