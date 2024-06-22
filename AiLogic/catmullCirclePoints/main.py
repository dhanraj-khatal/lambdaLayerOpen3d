import errno
import time

from AiLogic.catmullCirclePoints.helpers import *

MAX_STEPS_COUNT = 50
MILESTONE_COUNT = 1


logger = logging.getLogger()


def get_catmull_circle_points(
        initial_circles_folder: str,
        output_folder: str,
):
    start_time = time.time()

    if not os.path.isdir(initial_circles_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), initial_circles_folder)

    renew_folder(output_folder)

    # loop over json files present in initial_circles_folder
    for filename in os.listdir(initial_circles_folder):
        if not filename.endswith(".json"):
            continue

        # each file contains a list of points
        all_circle_points = get_json_object(os.path.join(initial_circles_folder, filename))
        logger.info(f'{all_circle_points=}')

        # get equidistant points on the segmentation line
        catmull_points = get_equidistant_circle_points(points=all_circle_points, no_of_equidistant_points_required=30)
        logger.info(f'{catmull_points=}')

        # assign mandibular circle files number greater than 16
        fileNameArray = filename.split("_")
        circleNumber = int(fileNameArray[3].split(".")[0])
        outputCircleNumber = circleNumber
        if circleNumber < 17 and fileNameArray[0] == "mandibular":
            mandibularCircleNumber = circleNumber + 16
            outputCircleNumber = mandibularCircleNumber - (((mandibularCircleNumber - 25)*2) +1)
        save_as_json(
            data={
                'circle_points': catmull_points
            },
            file_name=os.path.join(output_folder, f'{fileNameArray[0]}_tooth_circle_{outputCircleNumber}.json')
        )

    end_time = time.time()
    logger.info(f'Compute time taken: {(end_time - start_time)}')


def get_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--initial-circles-folder",
        "-icf",
        help="path to the initial circles folder containing the circle json for each tooth",
    )

    parser.add_argument(
        "--model-folder",
        "-mf",
        help="path to the model folder",
    )

    parser.add_argument(
        "--output-folder",
        "-of",
        help="path to save step GLTFs and other log/json files",
    )

    arguments = parser.parse_args()

    return arguments


def get_json_object(filepath: str):
    # Opening JSON file
    f = open(filepath)

    # returns JSON object as a dictionary
    json_obj = json.load(f)

    # Closing file
    f.close()

    return json_obj


def get_equidistant_circle_points(
        points: np.array,
        no_of_equidistant_points_required: int
) -> np.array:
    """
    Takes in a list of points to return a set of equidistant points of given length

    @param points: List of points
    @param no_of_equidistant_points_required: no of equidistant points required
    @return:
    """
    points = np.append(points, [points[0]], axis=0)
    n_points = len(points)
    pair_distances = []

    for i in range(0, n_points):
        first_point = np.asarray(points[i])
        second_point = np.asarray(points[(i + 1) % n_points])

        dist = np.linalg.norm(second_point - first_point)
        pair_distances.append(dist)

    total_circle_distance_length = np.sum(np.asarray(pair_distances))
    equidistant_points = []
    for j in range(5):
        required_segment_length = total_circle_distance_length / (no_of_equidistant_points_required + j)
        equidistant_points = [points[0]]

        current_segment_length = 0
        for i in range(n_points - 1):
            new_segment_length = current_segment_length + pair_distances[i]
            if new_segment_length < required_segment_length:
                current_segment_length = new_segment_length
                continue

            if abs(current_segment_length - required_segment_length) < abs(
                    new_segment_length - required_segment_length):
                equidistant_points.append(list(points[i]))
                current_segment_length = pair_distances[i]
            else:
                equidistant_points.append(list(points[i + 1]))
                current_segment_length = 0

        if len(equidistant_points) >= no_of_equidistant_points_required:
            break

    return equidistant_points[no_of_equidistant_points_required-1::-1]
