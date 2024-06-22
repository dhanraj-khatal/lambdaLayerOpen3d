import numpy as np
import json

def get_slope(point_1: np.array, point_2: np.array, to_norm: bool = True):
    '''
    Get slope between 2 points (direction from point_1 to point_2)

    @param point1: Point 1
    @param point2: Point 2
    @param to_norm: Whether to normalize the slope with length 1
    '''
    line = np.asarray((point_2 - point_1))
    if to_norm:
        line = line / (np.sqrt(np.sum(line ** 2)) + 0.0001)

    return line[0]


def axes_tool_modification(tooth_name,axes_tool_output, output_file_name):
    try:

        tooth_name = tooth_name  #from api req

        f = open(axes_tool_output)
        # returns JSON object as a dictionary
        data = json.load(f)

        center = np.array([[data["tooth_crown_center"]["_x"],data["tooth_crown_center"]["_y"],data["tooth_crown_center"]["_z"]]]) 
        top_ax = np.array([[data["top_point"]["_x"],data["top_point"]["_y"],data["top_point"]["_z"]]]) 
        side_ax = np.array([[data["side_point"]["_x"],data["side_point"]["_y"],data["side_point"]["_z"]]]) 
        front_ax = np.array([[data["front_point"]["_x"],data["front_point"]["_y"],data["front_point"]["_z"]]]) 

        top_axis_value = get_slope(center, top_ax)
        side_axis_value = get_slope(center, side_ax)
        front_axis_value = get_slope(center, front_ax)

        data["side_axis"] = {
            "_x":side_axis_value[0],
            "_y":side_axis_value[1],
            "_z":side_axis_value[2]
        }
        

        data["top_axis"] = {
            "_x":top_axis_value[0],
            "_y":top_axis_value[1],
            "_z":top_axis_value[2]
        }
        

        data["front_axis"] = {
            "_x":front_axis_value[0],
            "_y":front_axis_value[1],
            "_z":front_axis_value[2]
        }
        
        out_file = open(output_file_name, "w")
        json.dump(data, out_file, indent=4)
        out_file.close()
        return True
        

    except Exception as e:
        print('error: ',e)
        return False



# if __name__ == "__main__":

#     axes_tool_modification("tooth_2", "Tooth_2_axes (2).json", "hello.json")
