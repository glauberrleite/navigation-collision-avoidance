import osqp
import numpy
import scipy.sparse as sparse

class MPC_ORCA:

    def __init__(self, goal, position, v_min, v_max, N, Ts):
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
        Aeq = sparse.hstack([Ax, Bu])
        leq = numpy.hstack([-x0, numpy.zeros(N*self.nx)])
        ueq = leq

        # - input and state constraints
        Aineq = sparse.eye((N+1) * self.nx + N * self.nu)
        self.lineq = numpy.hstack([numpy.kron(numpy.ones(N+1), xmin), numpy.kron(numpy.ones(N), umin)])
        self.uineq = numpy.hstack([numpy.kron(numpy.ones(N+1), xmax), numpy.kron(numpy.ones(N), umax)])
        
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq]).tocsc()
        l = numpy.hstack([leq, self.lineq])
        u = numpy.hstack([ueq, self.uineq])

        # Setting problem
        self.problem = osqp.OSQP()
        self.problem.setup(P, q, A, l, u, warm_start=True)

    def getNewAcceleration(self, position, velocity):
        x0 = numpy.array([position[0], position[1], velocity[0], velocity[1]])
        leq = numpy.hstack([-x0, numpy.zeros(self.N * self.nx)])
        ueq = leq

        l_new = numpy.hstack([leq, self.lineq])
        u_new = numpy.hstack([ueq, self.uineq])

        self.problem.update(l=l_new, u=u_new)
        result = self.problem.solve()

        # return the first control input
        return result.x[(-self.N*self.nu):(-(self.N-1)*self.nu)]
