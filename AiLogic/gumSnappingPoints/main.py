import errno
import time

from AiLogic.gumSnappingPoints.helpers import *

MAX_STEPS_COUNT = 50
MILESTONE_COUNT = 1


def get_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--initial-folder",
        "-if",
        help="path to a particular case folder with initial teeth stls",
    )

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


def get_gum_snapping_points(
        initial_folder: str,
        initial_circles_folder: str,
        model_folder: str,
        output_folder: str,
):
    start_time = time.time()
    logger = logging.getLogger()

    if not os.path.isdir(initial_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), initial_folder)

    if not os.path.isdir(initial_circles_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), initial_circles_folder)

    if not os.path.isdir(model_folder):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), model_folder)

    renew_folder(output_folder)

    st = time.time()
    initial_teeth = Teeth(
        case_folder=initial_folder,
        circles_folder=initial_circles_folder,
        model_folder=model_folder,
    )
    logger.info(f'Initial teeth imported {time.time() - st}')

    # save teeth GLTFs per step
    initial_teeth.export_teeth_circle_points(teeth_circle_steps_path=output_folder)

    end_time = time.time()
    logger.info(f'Compute time taken: {(end_time - start_time)}')
