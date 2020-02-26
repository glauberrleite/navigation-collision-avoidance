import osqp
import numpy
import scipy.sparse as sparse
from pyorca import Agent, orca

class MPC_ORCA:

    def __init__(self, position, v_min, v_max, N, N_c, Ts, colliders, tau, robot_radius):
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
        self.N_c = N_c
        self.Ts = Ts
        self.tau = tau

        self.agent = Agent(position, numpy.zeros(2), numpy.zeros(2), robot_radius)
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
        xmin = numpy.array([-numpy.inf, -numpy.inf, -numpy.inf, -numpy.inf])
        xmax = numpy.array([numpy.inf, numpy.inf, numpy.inf, numpy.inf])
        umin = numpy.array([-numpy.inf, -numpy.inf])
        umax = numpy.array([numpy.inf, numpy.inf])

        # Initial state
        x_0 = numpy.array([position[0], position[1], 0., 0.])

        # Setpoint
        x_r = x_0

        # MPC objective function
        #Q_0 = sparse.diags([100.0, 100.0, 0.0, 0.0])
        Q_0 = sparse.diags([3, 3, 0.0, 0.0])
        Q = sparse.diags([1.5, 1.5, 0.0, 0.0])
        R = 1.5 * sparse.eye(self.nu)

        # Casting QP format
        # QP objective
        P = sparse.block_diag([Q, Q_0, sparse.kron(sparse.eye(N-1), Q), sparse.kron(sparse.eye(N), R)]).tocsc()
        self.q = numpy.hstack([-Q.dot(x_r), -Q_0.dot(x_r), numpy.kron(numpy.ones(N-1), -Q.dot(x_r)), numpy.zeros(N * self.nu)])

        # QP constraints
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(self.nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
        A_eq = sparse.hstack([Ax, Bu])
        l_eq = numpy.hstack([-x_0, numpy.zeros(N*self.nx)])
        u_eq = l_eq

        # - Control horizon constraint
        A_N_c = sparse.hstack([numpy.zeros((self.nu * (N - N_c), (N+1) * self.nx)), \
            numpy.zeros((self.nu * (N - N_c), (N_c - 1) * self.nu)), \
            -sparse.kron(numpy.ones(((N - N_c), 1)), sparse.eye(self.nu)), \
            sparse.eye(self.nu * (N - N_c))])
        l_N_c = numpy.zeros(self.nu * (N - N_c))
        u_N_c = numpy.zeros(self.nu * (N - N_c))

        # - input and state constraints
        A_ineq = sparse.eye((N+1) * self.nx + N * self.nu)
        l_ineq = numpy.hstack([numpy.kron(numpy.ones(N+1), xmin), numpy.kron(numpy.ones(N), umin)])
        u_ineq = numpy.hstack([numpy.kron(numpy.ones(N+1), xmax), numpy.kron(numpy.ones(N), umax)])
        
        # ORCA Constraints
        A_ORCA_data = numpy.zeros(2 * len(self.colliders) * self.N)
        A_ORCA_rows = numpy.zeros(2 * len(self.colliders) * self.N)
        A_ORCA_cols = numpy.zeros(2 * len(self.colliders) * self.N)
        cnt = 0
        for k in range(N):
            for i in range(len(colliders)):
                A_ORCA_rows[cnt] = i * N + k
                A_ORCA_cols[cnt] = self.nx + k * self.nx + 2

                A_ORCA_rows[cnt + 1] = i * N + k
                A_ORCA_cols[cnt + 1] = self.nx + k * self.nx + 3
                cnt += 2
        A_ORCA = sparse.csc_matrix((A_ORCA_data, (A_ORCA_rows, A_ORCA_cols)), shape=(len(colliders) * N, A_eq.shape[1]))
        l_ORCA = numpy.zeros(len(colliders) * N)
        u_ORCA = numpy.zeros(len(colliders) * N)

        # OSQP constraints
        self.A = sparse.vstack([A_eq, A_N_c, A_ineq, A_ORCA]).tocsc()
        self.l = numpy.hstack([l_eq, l_N_c, l_ineq, l_ORCA])
        self.u = numpy.hstack([u_eq, u_N_c, u_ineq, u_ORCA])
        self.Q_0 = Q_0
        self.Q = Q
        self.R = R

        self.orca_rows_idx = A_eq.shape[0] + A_N_c.shape[0] +  A_ineq.shape[0]

        # Setting problem
        self.problem = osqp.OSQP()
        self.problem.setup(P, self.q, self.A, self.l, self.u, warm_start=True, verbose=False)

    def compute(self, setpoint):
        # Updating initial conditions
        x_0 = numpy.array([self.agent.position[0], self.agent.position[1], self.agent.velocity[0], self.agent.velocity[1]])
        
        self.q = numpy.hstack([-self.Q.dot(setpoint[0:self.nx]), -self.Q_0.dot(setpoint[self.nx:2*self.nx]), numpy.dot(sparse.kron(sparse.eye(self.N-1), -self.Q).toarray(), setpoint[2*self.nx:]), numpy.zeros(self.N * self.nu)])
        
        self.l[:self.nx] = -x_0
        self.u[:self.nx] = -x_0

        # Predict future states with constant velocity, i.e. no acceleration
        for k in range(self.N):
            agent_k = Agent(self.agent.position + k * self.agent.velocity * self.Ts, self.agent.velocity, numpy.zeros(2), self.agent.radius)

            for i, collider in enumerate(self.colliders):
                collider_k = Agent(collider.position + k * collider.velocity * self.Ts, collider.velocity, numpy.zeros(2), collider.radius)

                # Discovering ORCA half-space
                v0, n = orca(agent_k, collider_k, self.tau, self.Ts)

                self.A[self.orca_rows_idx + i * self.N + k, self.nx + k * self.nx + 2] = n[0]
                self.A[self.orca_rows_idx + i * self.N + k, self.nx + k * self.nx + 3] = n[1]

                self.l[self.orca_rows_idx + i * self.N + k] = -numpy.inf
                self.u[self.orca_rows_idx + i * self.N + k] = numpy.dot(n, v0)

        self.problem.update(q=self.q, l=self.l, u=self.u, Ax=self.A.data)
        result = self.problem.solve()

        if result.info.status == 'solved':
            # return the first resulting velocity after control action
            return [result.x[(self.nx + 2):(self.nx + 4)], result.x[-self.N*self.nu:-(self.N-1)*self.nu]]
        else:
            print(result.info.status)
            
            damping = 3
            new_acceleration = (1 - damping) * self.agent.acceleration
            return [self.agent.velocity + new_acceleration * self.Ts, new_acceleration]