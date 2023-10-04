import pandas, sys
if len(sys.argv) < 1:
    print("need filename: python3 python3_parse_results.py filename")
    exit()
else:
    filename = sys.argv[1]

print(filename)

k = pandas.read_csv(filename, error_bad_lines=False)

if (k["controller"] == 'potential_gap').all():
    controller = "potential_gap"
elif (k["controller"] == 'teb').all():
    controller = "teb"
else:
    controller = None

if controller is None:
    print("More than one controller")
    exit()

environments = set(k['world'])
robots = set(k['robot'])
fovs = set(k['fov'])

for environ in environments:
    for robot in robots:
        for fov in fovs:
            key = (k['world'] == environ) & (k['robot'] == robot) & (k['fov'] == fov)
            g = k[key]
            sr = (g['result'] == "SUCCEEDED").sum() / float(len(g['result']))
            print("{} {} {} {:.2f} {:.2f} {:.2f}".format(environ, robot, fov, sr, g['path_length'].mean(), g['time'].mean() / 1e9))