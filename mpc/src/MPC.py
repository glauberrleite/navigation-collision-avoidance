import osqp
import numpy
import scipy.sparse as sparse

class MPC:

    def __init__(self, position, v_min, v_max, N, Ts):
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
        umin = numpy.array([v_min, v_min])
        umax = numpy.array([v_max, v_max])

        # Initial state
        self.x_0 = numpy.array([position[0], position[1], 0., 0.])

        # Setpoint
        x_r = self.x_0

        # MPC objective function
        Q = sparse.diags([1.5, 1.5, 1.0, 1.0])
        R = 0.2 * sparse.eye(self.nu)

        # Casting QP format
        # QP objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N+1), Q), sparse.kron(sparse.eye(N), R)]).tocsc()
        self.q = numpy.hstack([numpy.kron(numpy.ones(N+1), -Q.dot(x_r)), numpy.zeros(N * self.nu)])

        # QP constraints
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(self.nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
        A_eq = sparse.hstack([Ax, Bu])
        l_eq = numpy.hstack([-self.x_0, numpy.zeros(N*self.nx)])
        u_eq = l_eq

        # - input and state constraints
        A_ineq = sparse.eye((N+1) * self.nx + N * self.nu)
        l_ineq = numpy.hstack([numpy.kron(numpy.ones(N+1), xmin), numpy.kron(numpy.ones(N), umin)])
        u_ineq = numpy.hstack([numpy.kron(numpy.ones(N+1), xmax), numpy.kron(numpy.ones(N), umax)])

        # OSQP constraints
        A = sparse.vstack([A_eq, A_ineq]).tocsc()
        self.l = numpy.hstack([l_eq, l_ineq])
        self.u = numpy.hstack([u_eq, u_ineq])
        self.Q = Q
        self.R = R

        # Setting problem
        self.problem = osqp.OSQP()
        self.problem.setup(P, self.q, A, self.l, self.u, warm_start=True, verbose=False)

    def getNewVelocity(self, setpoint):
        # Updating initial conditions        
        self.q = numpy.hstack([numpy.dot(sparse.kron(sparse.eye(self.N+1), -self.Q).toarray(), setpoint), numpy.zeros(self.N * self.nu)])

        self.l[:self.nx] = -self.x_0
        self.u[:self.nx] = -self.x_0

        self.problem.update(q=self.q, l=self.l, u=self.u)
        result = self.problem.solve()

        if result.info.status == 'solved':
            # return the first resulting velocity after control action
            return [result.x[(self.nx + 2):(self.nx + 4)], result.x[-self.N*self.nu:-(self.N-1)*self.nu]]
        else:
            print('unsolved')
            return [numpy.array([self.x_0[2], self.x_0[3]]), numpy.zeros(2)]
            #damping = 0.05
            #return numpy.array([self.agent.velocity[0] - numpy.sign(self.agent.velocity[0])*damping, self.agent.velocity[1] - numpy.sign(self.agent.velocity[1])*damping])
        
    