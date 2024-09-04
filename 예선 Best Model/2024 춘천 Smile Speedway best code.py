import math

def dist(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

def rect(r, theta):
    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x, y

def polar(x, y):
    r = (x ** 2 + y ** 2) ** .5
    theta = math.degrees(math.atan2(y, x))
    return r, theta

def normalize_angle(angle):
    n = math.floor(angle / 360.0)
    angle_between_0_and_360 = angle - n * 360.0
    if angle_between_0_and_360 <= 180.0:
        return angle_between_0_and_360
    else:
        return angle_between_0_and_360 - 360

def get_shortcut_waypoints():
    return [[ 0.30673024,  0.82358544],
            [ 0.13304998,  1.01401461],
            [-0.03980582,  1.20439928],
            [-0.21219527,  1.39499802],
            [-0.38446241,  1.58614125],
            [-0.55710422,  1.77834161],
            [-0.73112335,  1.97273832],
            [-0.91030781,  2.17368691],
            [-1.10720367,  2.39570079],
            [-1.30881055,  2.61927559],
            [-1.51543331,  2.83425602],
            [-1.72894557,  3.03252805],
            [-1.94968713,  3.20655049],
            [-2.1763386 ,  3.34974416],
            [-2.40609511,  3.45652682],
            [-2.63486846,  3.52263592],
            [-2.85757564,  3.54532022],
            [-3.06773922,  3.52159076],
            [-3.25660935,  3.448586  ],
            [-3.40845032,  3.32114386],
            [-3.52312882,  3.15673059],
            [-3.60120478,  2.96517937],
            [-3.64326127,  2.75356585],
            [-3.65060857,  2.52796926],
            [-3.62478429,  2.29366503],
            [-3.56850026,  2.05591246],
            [-3.48693446,  1.82177705],
            [-3.39004408,  1.60171181],
            [-3.30965762,  1.38291901],
            [-3.24707751,  1.15940035],
            [-3.206793  ,  0.92988015],
            [-3.19594798,  0.69224852],
            [-3.20902886,  0.44780122],
            [-3.24303838,  0.19711634],
            [-3.29567622, -0.05955324],
            [-3.36442203, -0.32209618],
            [-3.44698196, -0.59073935],
            [-3.54035563, -0.86537299],
            [-3.62926913, -1.15360863],
            [-3.7035612 , -1.44407701],
            [-3.75660932, -1.73535261],
            [-3.78194598, -2.02375483],
            [-3.7747643 , -2.30341994],
            [-3.73222168, -2.5669412 ],
            [-3.65478422, -2.80696014],
            [-3.54484436, -3.0165817 ],
            [-3.40640585, -3.18981154],
            [-3.24454303, -3.32147723],
            [-3.0642818 , -3.40330425],
            [-2.87357392, -3.42515778],
            [-2.68780206, -3.36961484],
            [-2.51712019, -3.26061852],
            [-2.36697555, -3.10460527],
            [-2.24053081, -2.90858387],
            [-2.13831561, -2.68050749],
            [-2.05853872, -2.4281919 ],
            [-1.99507654, -2.16083002],
            [-1.91526773, -1.89760086],
            [-1.81486267, -1.65643115],
            [-1.69115698, -1.44600362],
            [-1.54517725, -1.27307704],
            [-1.38044352, -1.14223936],
            [-1.2015248 , -1.05723313],
            [-1.01353974, -1.02640618],
            [-0.82683999, -1.06360346],
            [-0.65155944, -1.153976  ],
            [-0.49474326, -1.29193598],
            [-0.36340789, -1.47328921],
            [-0.26387562, -1.69225592],
            [-0.20031639, -1.94100571],
            [-0.17398838, -2.21087213],
            [-0.18351302, -2.49369229],
            [-0.16513585, -2.72967863],
            [-0.11902761, -2.94272947],
            [-0.04353248, -3.13001795],
            [ 0.06279584, -3.28765937],
            [ 0.20220513, -3.4091118 ],
            [ 0.37908622, -3.48044494],
            [ 0.57830735, -3.51524557],
            [ 0.7922712 , -3.52298619],
            [ 1.01401186, -3.51545155],
            [ 1.26492918, -3.51569963],
            [ 1.51558255, -3.52345134],
            [ 1.76600558, -3.53755239],
            [ 2.01623035, -3.55700979],
            [ 2.26625838, -3.58169519],
            [ 2.50205192, -3.60996426],
            [ 2.73070817, -3.62147626],
            [ 2.94639687, -3.60601471],
            [ 3.14336065, -3.55737124],
            [ 3.3146531 , -3.47114769],
            [ 3.45126872, -3.34446228],
            [ 3.53406651, -3.1730729 ],
            [ 3.56524433, -2.97438713],
            [ 3.54443415, -2.76006509],
            [ 3.47530703, -2.54082596],
            [ 3.3624516 , -2.32485612],
            [ 3.21270031, -2.11741586],
            [ 3.03442811, -1.92024645],
            [ 2.83670363, -1.73193879],
            [ 2.62895009, -1.54878645],
            [ 2.40922042, -1.34719472],
            [ 2.1946466 , -1.14350886],
            [ 1.98545167, -0.93949761],
            [ 1.78192126, -0.73649094],
            [ 1.58396081, -0.53532512],
            [ 1.39109578, -0.33629586],
            [ 1.20274277, -0.1393713 ],
            [ 1.01821121,  0.05569553],
            [ 0.83688898,  0.2492032 ],
            [ 0.65820261,  0.44147713],
            [ 0.48163739,  0.63283654],
            [ 0.30673024,  0.82358544]]

def get_waypoints_ordered_in_driving_direction(params, waypoints):
    if params['is_reversed']:
        return list(reversed(waypoints))
    else:
        return waypoints

def up_sample(waypoints, factor):
    n = len(waypoints)
    return [
        [i / factor * waypoints[(j + 1) % n][0] + (1 - i / factor) * waypoints[j][0],
         i / factor * waypoints[(j + 1) % n][1] + (1 - i / factor) * waypoints[j][1]
         ]
        for j in range(n)
        for i in range(factor)
    ]

def get_target_point(params):
    waypoints = up_sample(get_waypoints_ordered_in_driving_direction(params, get_shortcut_waypoints()), 20)
    car = [params['x'], params['y']]

    distances = [dist(p, car) for p in waypoints]
    min_dist = min(distances)
    i_closest = distances.index(min_dist)

    waypoints_starting_with_closest = waypoints[i_closest:] + waypoints[:i_closest]

    r = params['track_width'] * 0.45

    is_inside = [dist(p, car) < r for p in waypoints_starting_with_closest]
    try:
        i_first_outside = is_inside.index(False)
    except ValueError:
        return waypoints[i_closest]

    return waypoints_starting_with_closest[i_first_outside]

def get_target_steering_degree(params):
    tx, ty = get_target_point(params)
    car_x = params['x']
    car_y = params['y']
    dx = tx - car_x
    dy = ty - car_y
    heading = params['heading']

    _, target_angle = polar(dx, dy)
    steering_angle = target_angle - heading

    return normalize_angle(steering_angle)

def score_steer_to_point_ahead(params):
    best_steering_angle = get_target_steering_degree(params)
    steering_angle = params['steering_angle']

    error = (steering_angle - best_steering_angle) / 60.0
    score = 1.0 - abs(error)

    return max(score, 0.01)

def reward_function(params):
    return float(score_steer_to_point_ahead(params))
