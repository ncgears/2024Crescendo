package frc.team1918.paths;

import frc.team1918.lib.control.SwerveTrajectory;

public class OneMeterBackward extends Path {
   private final static double[][] points = {
       {0,0.0,0.0,0.0,0.0,0.0,0.0},
       {0.0133,0.0,0.0,0.0,-0.0302,-0.0,-0.0},
       {0.0265,-0.0004,-0.0,-0.0,-0.0604,-0.0,-0.0},
       {0.0398,-0.0012,-0.0,-0.0,-0.0906,-0.0,-0.0},
       {0.053,-0.0024,-0.0,-0.0,-0.1208,-0.0,-0.0},
       {0.0663,-0.004,-0.0,-0.0,-0.1509,-0.0,-0.0},
       {0.0795,-0.006,-0.0,-0.0,-0.1811,-0.0,-0.0},
       {0.0928,-0.0084,-0.0,-0.0,-0.2113,-0.0,-0.0},
       {0.106,-0.0112,-0.0,-0.0,-0.2415,-0.0,-0.0},
       {0.1193,-0.0144,-0.0,-0.0,-0.2717,-0.0,-0.0},
       {0.1325,-0.018,-0.0,-0.0,-0.3019,-0.0,-0.0},
       {0.1458,-0.022,-0.0,-0.0,-0.3321,-0.0,-0.0},
       {0.159,-0.0264,-0.0,-0.0,-0.3623,-0.0,-0.0},
       {0.1723,-0.0312,-0.0,-0.0,-0.3925,-0.0,-0.0},
       {0.1855,-0.0364,-0.0,-0.0,-0.4226,-0.0,-0.0},
       {0.1988,-0.042,-0.0,-0.0,-0.4528,-0.0,-0.0},
       {0.212,-0.048,-0.0,-0.0,-0.483,-0.0,-0.0},
       {0.2253,-0.0544,-0.0,-0.0,-0.5132,-0.0,-0.0},
       {0.2385,-0.0612,-0.0,-0.0,-0.5434,-0.0,-0.0},
       {0.2518,-0.0684,-0.0,-0.0,-0.5736,-0.0,-0.0},
       {0.265,-0.076,-0.0,-0.0,-0.6038,-0.0,-0.0},
       {0.2783,-0.084,-0.0,-0.0,-0.634,-0.0,-0.0},
       {0.2915,-0.0924,-0.0,-0.0,-0.6641,-0.0,-0.0},
       {0.3048,-0.1012,-0.0,-0.0,-0.6943,-0.0,-0.0},
       {0.318,-0.1104,-0.0,-0.0,-0.7245,-0.0,0.0},
       {0.3313,-0.12,-0.0,-0.0,-0.7547,-0.0,-0.0},
       {0.3445,-0.13,-0.0,-0.0,-0.7849,-0.0,0.0},
       {0.3578,-0.1404,-0.0,-0.0,-0.8151,-0.0,-0.0},
       {0.371,-0.1512,-0.0,-0.0,-0.8453,-0.0,0.0},
       {0.3843,-0.1624,-0.0,-0.0,-0.8755,-0.0,-0.0},
       {0.3975,-0.174,-0.0,-0.0,-0.9057,-0.0,-0.0},
       {0.4108,-0.186,-0.0,-0.0,-0.9358,-0.0,-0.0},
       {0.424,-0.1984,-0.0,-0.0,-0.966,-0.0,-0.0},
       {0.4373,-0.2112,-0.0,-0.0,-0.9962,-0.0,-0.0},
       {0.4505,-0.2244,-0.0,-0.0,-1.0264,-0.0,-0.0},
       {0.4638,-0.238,-0.0,-0.0,-1.0566,-0.0,0.0},
       {0.477,-0.252,-0.0,-0.0,-1.0868,-0.0,0.0},
       {0.4903,-0.2664,-0.0,-0.0,-1.117,-0.0,0.0},
       {0.5035,-0.2812,-0.0,-0.0,-1.1472,-0.0,0.0},
       {0.5168,-0.2964,-0.0,-0.0,-1.1774,-0.0,0.0},
       {0.53,-0.312,-0.0,-0.0,-1.2075,-0.0,0.0},
       {0.5433,-0.328,-0.0,-0.0,-1.2377,-0.0,0.0},
       {0.5565,-0.3444,-0.0,-0.0,-1.2679,-0.0,0.0},
       {0.5698,-0.3612,-0.0,-0.0,-1.2981,-0.0,0.0},
       {0.583,-0.3784,-0.0,-0.0,-1.3283,-0.0,0.0},
       {0.5963,-0.396,-0.0,-0.0,-1.3585,-0.0,0.0},
       {0.6095,-0.414,-0.0,-0.0,-1.3887,-0.0,0.0},
       {0.6228,-0.4324,-0.0,-0.0,-1.4189,-0.0,0.0},
       {0.636,-0.4512,-0.0,-0.0,-1.449,-0.0,0.0},
       {0.6493,-0.4704,-0.0,-0.0,-1.4792,-0.0,0.0},
       {0.6625,-0.49,-0.0,-0.0,-1.5094,0.0,0.0},
       {0.6758,-0.51,-0.0,-0.0,-1.4792,0.0,0.0},
       {0.689,-0.5296,-0.0,-0.0,-1.449,0.0,0.0},
       {0.7023,-0.5488,-0.0,-0.0,-1.4189,0.0,0.0},
       {0.7155,-0.5676,-0.0,0.0,-1.3887,0.0,0.0},
       {0.7288,-0.586,-0.0,0.0,-1.3585,0.0,0.0},
       {0.742,-0.604,-0.0,0.0,-1.3283,0.0,0.0},
       {0.7553,-0.6216,-0.0,0.0,-1.2981,0.0,0.0},
       {0.7685,-0.6388,-0.0,0.0,-1.2679,0.0,0.0},
       {0.7818,-0.6556,-0.0,0.0,-1.2377,0.0,0.0},
       {0.795,-0.672,-0.0,0.0,-1.2075,0.0,0.0},
       {0.8083,-0.688,-0.0,0.0,-1.1774,0.0,0.0},
       {0.8215,-0.7036,-0.0,0.0,-1.1472,0.0,0.0},
       {0.8348,-0.7188,-0.0,0.0,-1.117,0.0,0.0},
       {0.848,-0.7336,-0.0,0.0,-1.0868,0.0,0.0},
       {0.8613,-0.748,-0.0,0.0,-1.0566,0.0,0.0},
       {0.8745,-0.762,-0.0,0.0,-1.0264,0.0,-0.0},
       {0.8878,-0.7756,-0.0,0.0,-0.9962,0.0,-0.0},
       {0.901,-0.7888,-0.0,0.0,-0.966,0.0,-0.0},
       {0.9143,-0.8016,-0.0,0.0,-0.9358,0.0,-0.0},
       {0.9275,-0.814,-0.0,0.0,-0.9057,0.0,-0.0},
       {0.9408,-0.826,-0.0,0.0,-0.8755,0.0,0.0},
       {0.954,-0.8376,-0.0,0.0,-0.8453,0.0,0.0},
       {0.9673,-0.8488,-0.0,0.0,-0.8151,0.0,0.0},
       {0.9805,-0.8596,-0.0,0.0,-0.7849,0.0,0.0},
       {0.9938,-0.87,-0.0,0.0,-0.7547,0.0,0.0},
       {1.007,-0.88,-0.0,0.0,-0.7245,0.0,0.0},
       {1.0203,-0.8896,-0.0,0.0,-0.6943,0.0,-0.0},
       {1.0335,-0.8988,-0.0,0.0,-0.6641,0.0,-0.0},
       {1.0468,-0.9076,-0.0,0.0,-0.634,0.0,-0.0},
       {1.06,-0.916,-0.0,0.0,-0.6038,0.0,-0.0},
       {1.0733,-0.924,-0.0,0.0,-0.5736,0.0,-0.0},
       {1.0865,-0.9316,-0.0,0.0,-0.5434,0.0,-0.0},
       {1.0998,-0.9388,-0.0,0.0,-0.5132,0.0,-0.0},
       {1.113,-0.9456,-0.0,0.0,-0.483,0.0,-0.0},
       {1.1263,-0.952,-0.0,0.0,-0.4528,0.0,-0.0},
       {1.1395,-0.958,-0.0,0.0,-0.4226,0.0,-0.0},
       {1.1528,-0.9636,-0.0,0.0,-0.3925,0.0,-0.0},
       {1.166,-0.9688,-0.0,0.0,-0.3623,0.0,-0.0},
       {1.1793,-0.9736,-0.0,0.0,-0.3321,0.0,-0.0},
       {1.1925,-0.978,-0.0,0.0,-0.3019,0.0,-0.0},
       {1.2058,-0.982,-0.0,0.0,-0.2717,0.0,-0.0},
       {1.219,-0.9856,-0.0,0.0,-0.2415,0.0,-0.0},
       {1.2323,-0.9888,-0.0,0.0,-0.2113,0.0,-0.0},
       {1.2455,-0.9916,-0.0,0.0,-0.1811,0.0,-0.0},
       {1.2588,-0.994,-0.0,0.0,-0.1509,0.0,-0.0},
       {1.272,-0.996,-0.0,0.0,-0.1208,0.0,-0.0},
       {1.2853,-0.9976,-0.0,0.0,-0.0906,0.0,-0.0},
       {1.2985,-0.9988,-0.0,0.0,-0.0604,0.0,-0.0},
       {1.3118,-0.9996,-0.0,0.0,-0.0302,0.0,-0.0},
       {1.325,-1.0,0.0,0.0,0.0,0.0,0.0},
   };
   public SwerveTrajectory getPath() {
       return new SwerveTrajectory(points);
   }
}
