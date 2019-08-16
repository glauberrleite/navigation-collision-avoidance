import osqp
import numpy
import scipy.sparse as sparse

class MPC_ORCA:

    def __init__(self, goal, position, v_min, v_max, N, Ts, n_colliders = 0):
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
        
        # ORCA Constraints
        A_ORCA = sparse.csc_matrix((n_colliders, A_eq.shape[1]))
        l_ORCA = numpy.zeros(n_colliders)
        u_ORCA = numpy.zeros(n_colliders)

        # OSQP constraints
        self.A = sparse.vstack([A_eq, A_ineq, A_ORCA]).tocsc()
        self.l = numpy.hstack([l_eq, l_ineq, l_ORCA])
        self.u = numpy.hstack([u_eq, u_ineq, u_ORCA])

        # Setting problem
        self.problem = osqp.OSQP()
        self.problem.setup(P, q, self.A, self.l, self.u, warm_start=True)

        # Saving non-ORCA A rows size for ORCA constraints updates
        rows_eq = (self.N + 1) * self.nx
        rows_ineq = (self.N + 1) * self.nx + self.N * self.nu
        self.rows_A = rows_eq + rows_ineq

    def getNewAcceleration(self, position, velocity):
        x0 = numpy.array([position[0], position[1], velocity[0], velocity[1]])
        
        self.l[:self.nx] = -x0
        self.u[:self.nx] = -x0

        self.problem.update(l=self.l, u=self.u)
        result = self.problem.solve()

        # return the first control input
        return result.x[(-self.N*self.nu):(-(self.N-1)*self.nu)]
