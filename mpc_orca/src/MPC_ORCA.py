import osqp
import numpy
import scipy.sparse as sparse
from pyorca import Agent, orca

class MPC_ORCA:

    def __init__(self, goal, position, v_min, v_max, N, Ts, colliders, tau, robot_radius):
        """ MPC-ORCA controller instance
        
        :param goal: Goal position
        :type goal: Numpy Array 2x1
        :param position: Initial position
        :type position: Numpy Array 2x1
        :param v_min: Lower velocity constraint
        :type v_min: float
        :param v_max: Upper velocity constraint
        :type v_max: float
        :param N: Prediction Horizon
        :type N: int
        :param Ts: Sampling Time
        :type Ts: float
        :returns: Controller instance
        :type: MPCORCA
        """

        self.N = N
        self.Ts = Ts
        self.tau = tau

        self.agent = Agent(position, [0., 0.], robot_radius)
        self.colliders = colliders

        # Linear Dynamics
        # x = [p_x, p_y, v_x, v_y]'
        # u = [a_x, a_y]'
        Ad = sparse.csc_matrix([
            [1.,    0., Ts, 0.  ],
            [0.,    1., 0., Ts  ],
            [0.,    0., 1., 0.  ],
            [0.,    0., 0., 1.  ]
        ])

        Bd = sparse.csc_matrix([
            [0.5 * Ts ** 2, 0.              ],
            [0.,            0.5 * Ts ** 2   ],
            [Ts,            0.              ],
            [0.,            Ts              ]
        ])
        [self.nx, self.nu] = Bd.shape

        # State constraints
        xmin = numpy.array([-numpy.inf, -numpy.inf, v_min, v_min])
        xmax = numpy.array([numpy.inf, numpy.inf, v_max, v_max])
        umin = numpy.array([-numpy.inf, -numpy.inf])
        umax = numpy.array([numpy.inf, numpy.inf])

        # Initial state
        x0 = numpy.array([position[0], position[1], 0., 0.])

        # Setpoint
        x_r = numpy.array([goal[0], goal[1], 0., 0.])

        # MPC objective function
        Q = sparse.diags([1., 1., 0., 0.])
        Q_n = Q
        R = 0.1*sparse.eye(self.nu)

        # Casting QP format
        # QP objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), Q_n, sparse.kron(sparse.eye(N), R)]).tocsc()
        q = numpy.hstack([numpy.kron(numpy.ones(N), -Q.dot(x_r)), -Q_n.dot(x_r), numpy.zeros(N * self.nu)])

        # QP constraints
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(self.nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
        A_eq = sparse.hstack([Ax, Bu])
        l_eq = numpy.hstack([-x0, numpy.zeros(N*self.nx)])
        u_eq = l_eq

        # - input and state constraints
        A_ineq = sparse.eye((N+1) * self.nx + N * self.nu)
        l_ineq = numpy.hstack([numpy.kron(numpy.ones(N+1), xmin), numpy.kron(numpy.ones(N), umin)])
        u_ineq = numpy.hstack([numpy.kron(numpy.ones(N+1), xmax), numpy.kron(numpy.ones(N), umax)])
        
        # Saving where ORCA rows start on A, l and u matrices
        rows_eq = (self.N + 1) * self.nx
        rows_ineq = (self.N + 1) * self.nx + self.N * self.nu
        self.orca_rows = rows_eq + rows_ineq

        # ORCA Constraints
        A_ORCA_data = numpy.zeros(2 * len(self.colliders) * self.N)
        A_ORCA_rows = numpy.zeros(2 * len(self.colliders) * self.N)
        A_ORCA_cols = numpy.zeros(2 * len(self.colliders) * self.N)
        for k in range(N):
            for i in range(len(colliders)):
                A_ORCA_rows[i * N + 2 * k] = i * N + k
                A_ORCA_cols[i * N + 2 * k] = self.nx + k * self.nx + 2

                A_ORCA_rows[i * N + 2 * k + 1] = i * N + k
                A_ORCA_cols[i * N + 2 * k + 1] = self.nx + k * self.nx + 3
        A_ORCA = sparse.csc_matrix((A_ORCA_data, (A_ORCA_rows, A_ORCA_cols)), shape=(len(colliders) * N, A_eq.shape[1]))
        l_ORCA = numpy.zeros(len(colliders) * N)
        u_ORCA = numpy.zeros(len(colliders) * N)

        # OSQP constraints
        self.A = sparse.vstack([A_eq, A_ineq, A_ORCA]).tocsc()
        self.l = numpy.hstack([l_eq, l_ineq, l_ORCA])
        self.u = numpy.hstack([u_eq, u_ineq, u_ORCA])

        # Setting problem
        self.problem = osqp.OSQP()
        self.problem.setup(P, q, self.A, self.l, self.u, warm_start=True)

    def getNewVelocity(self):
        # Updating initial conditions
        x0 = numpy.array([self.agent.position[0], self.agent.position[1], self.agent.velocity[0], self.agent.velocity[1]])
        
        self.l[:self.nx] = -x0
        self.u[:self.nx] = -x0

        A_new_data = numpy.zeros(2 * len(self.colliders) * self.N)
        A_new_idx = numpy.zeros(((2 * len(self.colliders) * self.N), 2))

        # Predict future states with constant velocity, i.e. no acceleration
        for k in range(self.N):
            agent_k = Agent(self.agent.position + k * self.agent.velocity * self.Ts, self.agent.velocity, self.agent.radius)

            for i, collider in enumerate(self.colliders):    
                collider_k = Agent(collider.position + k * collider.velocity * self.Ts, collider.velocity, collider.radius)

                # Dicovering ORCA half-space
                v0, n = orca(agent_k, collider_k, self.tau, self.Ts)

                A_new_idx[k * len(self.colliders) + 2 * i, 0] = self.orca_rows + i * self.N + k
                A_new_idx[k * len(self.colliders) + 2 * i, 1] = (self.nx + k * self.nx + 2)
                A_new_data[k * len(self.colliders) + 2 * i] = n[0]

                A_new_idx[k * len(self.colliders) + 2 * i + 1, 0] = self.orca_rows + i * self.N + k
                A_new_idx[k * len(self.colliders) + 2 * i + 1, 1] = (self.nx + k * self.nx + 3)
                A_new_data[k * len(self.colliders) + 2 * i + 1] = n[1]
                #self.A[self.orca_rows + i * self.N + k, (self.nx + k * self.nx + 2)] = n[0]
                #self.A[self.orca_rows + i * self.N + k, (self.nx + k * self.nx + 3)] = n[1]
                self.l[self.orca_rows + i * self.N + k] = -numpy.inf
                self.u[self.orca_rows + i * self.N + k] = numpy.dot(n, v0)
                #self.u[self.orca_rows + i * self.N + k] = numpy.inf
                #self.l[self.orca_rows + i * self.N + k] = numpy.dot(n, v0)
                

        self.problem.update(l=self.l, u=self.u, Ax=A_new_data, Ax_idx=A_new_idx)
        #self.problem.update(l=self.l, u=self.u)
        result = self.problem.solve()

        # return the first resulting velocity after control action
        return result.x[(self.nx + 2):(self.nx + 4)]
    