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
    return [[-3.53490617, -0.04164061],
            [-3.55675269, -0.21069381],
            [-3.55706974, -0.38237916],
            [-3.53696616, -0.55561264],
            [-3.49643095, -0.72943705],
            [-3.43485943, -0.90279568],
            [-3.35126422, -1.0744257 ],
            [-3.24080094, -1.24205378],
            [-3.09481526, -1.40053149],
            [-2.90172588, -1.53672613],
            [-2.69142477, -1.66152762],
            [-2.46703669, -1.77659114],
            [-2.23058595, -1.88358365],
            [-1.98412615, -1.98477969],
            [-1.72805329, -2.08099537],
            [-1.46710454, -2.16954877],
            [-1.20508633, -2.24859971],
            [-0.94277812, -2.31739392],
            [-0.68049824, -2.37521034],
            [-0.41850683, -2.42061897],
            [-0.15712608, -2.45177479],
            [ 0.10323054, -2.46679102],
            [ 0.3618151 , -2.46167282],
            [ 0.61737683, -2.43153477],
            [ 0.86773578, -2.37052539],
            [ 1.11205588, -2.28261396],
            [ 1.34993245, -2.17144665],
            [ 1.58083891, -2.03922435],
            [ 1.80402271, -1.88733415],
            [ 2.01869954, -1.71722443],
            [ 2.22411466, -1.53053924],
            [ 2.41955856, -1.3290534 ],
            [ 2.60437853, -1.1145788 ],
            [ 2.77794515, -0.88885545],
            [ 2.93976938, -0.65363624],
            [ 3.0890637 , -0.41036931],
            [ 3.22504348, -0.1604768 ],
            [ 3.34600355,  0.09493826],
            [ 3.4502179 ,  0.354499  ],
            [ 3.53547371,  0.61666045],
            [ 3.59876213,  0.87954236],
            [ 3.6368299 ,  1.14067515],
            [ 3.64596464,  1.39682271],
            [ 3.62310359,  1.64413837],
            [ 3.56275966,  1.87677955],
            [ 3.4584461 ,  2.08545928],
            [ 3.30602451,  2.25616401],
            [ 3.1193225 ,  2.3874289 ],
            [ 2.90971463,  2.48149902],
            [ 2.68556009,  2.54267097],
            [ 2.45192798,  2.57364934],
            [ 2.21252784,  2.57713681],
            [ 1.9699932 ,  2.55525016],
            [ 1.72629982,  2.51031004],
            [ 1.48290038,  2.44427835],
            [ 1.24088807,  2.35912021],
            [ 1.00108873,  2.25693797],
            [ 0.76419314,  2.13999366],
            [ 0.53101011,  2.01072264],
            [ 0.3026016 ,  1.87201036],
            [ 0.08052213,  1.72679055],
            [-0.14070758,  1.59089617],
            [-0.36533785,  1.4635226 ],
            [-0.59482026,  1.34777925],
            [-0.83070647,  1.24732181],
            [-1.07196871,  1.15994998],
            [-1.31837711,  1.08540534],
            [-1.56977429,  1.02369814],
            [-1.82595469,  0.97498121],
            [-2.08646093,  0.93957414],
            [-2.33147742,  0.89233585],
            [-2.56369888,  0.83150743],
            [-2.78045912,  0.75544647],
            [-2.97928827,  0.6634704 ],
            [-3.15663732,  0.55470585],
            [-3.30770178,  0.42856048],
            [-3.41529675,  0.28143114],
            [-3.48907217,  0.12323365],
            [-3.53490617, -0.04164061]]

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
