import numpy as np
import matplotlib.pyplot as plt
from   mpl_toolkits.mplot3d import Axes3D
from   math import pow, sqrt
import json

def axes_tool_modification(tooth_name,axes_tool_output, output_file_name):
    try:
        print(output_file_name)
        tooth_name = tooth_name  #from api req
        f = open(axes_tool_output)
        
        # returns JSON object as 
        # a dictionary
        data = json.load(f)
        print(data)
        f.close()

        #f1 = open("Tooth_2_axes.json")
        #AI_data = json.load(f1)
        #f1.close()
        #print(AI_data["tooth_center"]["_x"],"data")
        #axis_edit_output = {new_update_third_point:{_isDirty:true,_x:41.26211927831173,_y:-6.193291485309601,_z:21.211208283901215},new_update_crown_center_point:{_isDirty:true,_x:42.337108597159386,_y:-4.40401417016983,_z:26.060207307338715},new_update_front_surface_point:{_isDirty:true,_x:38.45713423192501,_y:-2.156661331653595,_z:25.49833482503891}}
        #print(axis_edit_output,"json")
        def lin(z,c_xz,m_xz,c_yz,m_yz):
            x = (z - c_xz)/m_xz
            y = (z - c_yz)/m_yz
            print(x,y)
            return x,y
        def slope(pts,tooth_name):
            #pts = np.add.accumulate(np.random.random((10,3)))
            #pts = np.array([[35.2576186543916,0.034435665891151886,-28.65065290709688],[45.538918428123,0.12626493349671364,24.36187119781971]])
            print(pts,type(pts),"pts")
            x, y, z = pts.T
            # plane parallel to the y-axis
            A_xz = np.vstack((x, np.ones(len(x)))).T
            m_xz, c_xz = np.linalg.lstsq(A_xz, z, rcond=None)[0]
            # plane parallel to the x-axis
            A_yz = np.vstack((y, np.ones(len(y)))).T
            m_yz, c_yz = np.linalg.lstsq(A_yz, z, rcond=None)[0]
            # the intersection of those two planes and  
            # the function for the line would be:
            # z = m_yz * y + c_yz
            # z = m_xz * x + c_xz
            # or:
            # get 2 points on the intersection line 
            za = z[0]
            zb = z[len(z) - 1]
            xa, ya = lin(za,c_xz,m_xz,c_yz,m_yz)
            xb, yb = lin(zb,c_xz,m_xz,c_yz,m_yz)
            # get distance between points
            len1 = sqrt(pow(xb - xa, 2) + pow(yb - ya, 2) + pow(zb - za, 2))
            # get slopes (projections onto x, y and z planes)
            sx = (xb - xa) / len1  # x slope
            sy = (yb - ya) / len1  # y slope
            sz = (zb - za) / len1  # z slope
            
            # integrity check - the sum of squares of slopes should equal 1.0
            # print (pow(sx, 2) + pow(sy, 2) + pow(sz, 2))
            #print(sx,sy,sz,"sx,sy,sz")
            '''fig = plt.figure()
            ax = Axes3D(fig)
            ax.set_xlabel("x, slope: %.4f" %sx, color='blue')
            ax.set_ylabel("y, slope: %.4f" %sy, color='blue')
            ax.set_zlabel("z, slope: %.4f" %sz, color='blue')
            ax.scatter(x, y, z)
            ax.plot([xa], [ya], [za], markerfacecolor='k', markeredgecolor='k', marker = 'o')
            ax.plot([xb], [yb], [zb], markerfacecolor='k', markeredgecolor='k', marker = 'o')
            ax.plot([xa, xb], [ya, yb], [za, zb], color = 'r')

            plt.show()'''
            return sx,sy,sz
        
        #l1_x,l1_y,l1_z = slope(np.array([[35.2576186543916,0.034435665891151886,-28.65065290709688],[44.57000828161836,2.7499532848596573,23.56811983883381]]))
        #l2_x,l2_y,l2_z = slope(np.array([[35.2576186543916,0.034435665891151886,-28.65065290709688],[44.319722179323435,-2.50777767598629,23.971823528409004]]))
        #l3_x,l3_y,l3_z = slope(np.array([[35.2576186543916,0.034435665891151886,-28.65065290709688],[40.3869867362082,-1.9391090720891953,21.82001383602619]]))

        l1_x,l1_y,l1_z = slope(np.array([[data["tooth_center"]["_x"],data["tooth_center"]["_y"],data["tooth_center"]["_z"]],[data["third_point"]["_x"],data["third_point"]["_y"],data["third_point"]["_z"]]]),tooth_name)

        l2_x,l2_y,l2_z = slope(np.array([[data["tooth_center"]["_x"],data["tooth_center"]["_y"],data["tooth_center"]["_z"]],[data["crown_center_point"]["_x"],data["crown_center_point"]["_y"],data["crown_center_point"]["_z"]]]),tooth_name)

        l3_x,l3_y,l3_z = slope(np.array([[data["tooth_center"]["_x"],data["tooth_center"]["_y"],data["tooth_center"]["_z"]],[data["front_surface_point"]["_x"],data["front_surface_point"]["_y"],data["front_surface_point"]["_z"]]]),tooth_name)
        
        # data["center_to_third_point_axis"]["_x"] = l1_x
        center_to_third_point_axis = {
            "_x":l1_x,
            "_y":l1_y,
            "_z":l1_z
        }
        center_to_crown_axis = {
            "_x":l2_x,
            "_y":l2_y,
            "_z":l2_z
        }
        center_to_front_point_axis = {
            "_x":l3_x,
            "_y":l3_y,
            "_z":l3_z
        }
        data["center_to_third_point_axis"] = center_to_third_point_axis
        data["center_to_crown_axis"] = center_to_crown_axis
        data["center_to_front_point_axis"] = center_to_front_point_axis
        json.dumps(data)
        print("center_to_third_point_axis", l1_x,l1_y,l1_z)


        print("center_to_crown_axis", l2_x,l2_y,l2_z)

        print("center_to_front_point_axis", l3_x,l3_y,l3_z)

        out_file = open(output_file_name, "w")
        json.dump(data, out_file, indent=4)
        out_file.close()
        return True
            #slope()
        '''fig = plt.figure()
        ax = Axes3D(fig)
        ax.set_xlabel("x, slope: %.4f" %sx, color='blue')
        ax.set_ylabel("y, slope: %.4f" %sy, color='blue')
        ax.set_zlabel("z, slope: %.4f" %sz, color='blue')
        ax.scatter(x, y, z)
        ax.plot([xa], [ya], [za], markerfacecolor='k', markeredgecolor='k', marker = 'o')
        ax.plot([xb], [yb], [zb], markerfacecolor='k', markeredgecolor='k', marker = 'o')
        ax.plot([xa, xb], [ya, yb], [za, zb], color = 'r')

        plt.show()'''

    except Exception as e:
        print('error: ',e)
        return False
