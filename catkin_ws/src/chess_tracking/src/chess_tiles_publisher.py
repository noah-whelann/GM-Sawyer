#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from chess_tracking.msg import StringAndFloatsGrid


#find where one corner is on the board in terms of pixels
#transform that into coordinates relative to the webcam
#find a transformation from the webcam to the base of the robot
#move the arm to that coordinate

class ChessTileCoordinates:
    def __init__(self):
        rospy.init_node('chess_tile_coordinates', anonymous=True)

        self.web_cam_sub = rospy.Subscriber("/logitech_c920/image_raw", Image, self.image_callback)

        self.camera_transform_sub = rospy.Subscriber("transformed_coordinates", Point, self.tile_coords_callback)

        self.image_pub = rospy.Publisher("processed_cam", Image, queue_size=10)

        self.coord_pub = rospy.Publisher("tile_coordinates", StringAndFloatsGrid, queue_size=10)

        print("hi")
        rospy.spin()
    
    def tile_coords_callback(self, msg):

        pass


    def image_callback(self, msg):
        print("callback")

        processed_img, corners = self.process_board(msg)

        self.image_pub.publish(processed_img)
        
        #A1 [0.439, -0.061, -0.156]
        #H1 [0.835, -0.005, -0.161]
        #H8 [0.775, 0.391, -0.146]
        #A8 [0.393, 0.336, -0.136]

        # grid_locations = transform_corners(corners)

        print(corners)
        
        # self.coord_pub.publish(grid_locations)

    # def transform_corners(self, corners):

        # return_grid = StringAndFloatsGrid()

        # grid_letters = ["A", "B", "C", "D", "E", "F", "G", "H"]
        # grid_numbers  = [1, 2, 3, 4, 5, 6, 7, 8]
        
        # print("First corner: ", )

        # for i in range(8):
        #     for j in range(8):
        #         return_grid[i][j].name = grid_letters[i] + str(grid_numbers[j]) # assign a1 -> a8 and onwards for the entire chess board
        #         return_grid[i][j].x = 
        #         return_grid[i][j].y
        #         return_grid[i][j].z
                

                

        # return return_grid

    def process_board(self, img):

        bridge = CvBridge()

        image = bridge.imgmsg_to_cv2(img, "bgr8")

        # Convert the image from BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(hsv, (7,7),None)

        if ret:
            corners = corners.squeeze()

            all_corners = np.empty((0, 2))

            # top left corner
            corner = corners[0] + corners[0] - corners[8]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            # top right corner
            corner = corners[6] + corners[6] - corners[12]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            # bottom left corner
            corner = corners[42] + corners[42] - corners[36]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
            
            # bottom right corner
            corner = corners[48] + corners[48] - corners[40]
            cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
            all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            for i in range(49):
                color = (0, 255, 0)
                if i // 7 == 0: # first row
                    corner = 2 * corners[i] - corners[i+7]
                    cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                    all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
                
                elif i // 7 == 6:
                    corner = 2 * corners[i] - corners[i-7]
                    cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                    all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)
                
                if i % 7 == 0: # left edge
                    corner = 2 * corners[i] - corners[i+1]
                    cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                    all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

                if i % 7 == 6: # right edge
                    corner = 2 * corners[i] - corners[i-1]
                    cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                    all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

                corner = corners[i]
                cv2.circle(image, (int(corner[0]), int(corner[1])), 30, (0, 0, 255), 4)  # Draw the circle
                all_corners = np.append(all_corners, corner.reshape(1, -1), axis=0)

            all_corners = all_corners.tolist()

        else:
            print("NOT FOUND")
            corner = [0, 0]
        
        return bridge.cv2_to_imgmsg(image), all_corners

if __name__ == "__main__":
    ChessTileCoordinates()

'''
49 coords 
[[ 647.5286   137.4259 ]
 [ 771.8964   142.6874 ]
 [ 898.0685   147.61621]
 [1020.99976  149.65016]
 [1146.3932   156.35721]
 [1273.0488   160.55772]
 [1401.0702   163.33093]
 [ 642.6213   263.86096]
 [ 768.0286   269.53705]
 [ 891.7468   274.9615 ]
 [1016.1571   278.89566]
 [1140.5575   286.44254]
 [1266.2832   288.0092 ]
 [1392.0369   291.48767]
 [ 639.18896  389.41196]
 [ 762.5676   394.7501 ]
 [ 886.43756  400.86267]
 [1010.54614  405.98755]
 [1135.0403   410.0612 ]
 [1258.775    413.6757 ]
 [1386.0175   419.0489 ]
 [ 635.1926   513.32666]
 [ 758.72437  518.78015]
 [ 881.63495  524.5155 ]
 [1004.4566   530.75446]
 [1129.5245   535.415  ]
 [1254.2351   539.6399 ]
 [1377.8043   543.4602 ]
 [ 631.85284  635.8606 ]
 [ 754.0357   641.80707]
 [ 877.0244   646.96313]
 [1000.1061   652.81885]
 [1123.6398   657.7821 ]
 [1247.727    663.02563]
 [1372.0232   668.24347]
 [ 629.4015   758.1759 ]
 [ 750.27527  763.25104]
 [ 872.5131   768.7219 ]
 [ 994.85443  774.24976]
 [1118.0385   780.0498 ]
 [1240.9467   785.4582 ]
 [1365.5676   790.9377 ]
 [ 625.21375  877.5205 ]
 [ 746.4008   884.1761 ]
 [ 868.2948   889.57733]
 [ 989.49615  894.57495]
 [1111.9459   900.708  ]
 [1235.5393   906.8279 ]
 [1359.1764   913.14154]]



 [[546.5472412109375, 18.122955322265625], [1553.020263671875, 60.0533447265625], [505.2271728515625, 1004.2926635742188], [1486.28564453125, 1065.3271484375], [670.6596069335938, 24.98480224609375], [540.8052978515625, 144.1320037841797], [665.1450805664062, 149.99072265625], [795.226806640625, 29.84039306640625], [789.48486328125, 155.8494415283203], [920.4722290039062, 36.161834716796875], [914.0362548828125, 162.06967163085938], [1041.7049560546875, 42.52813720703125], [1037.0, 168.0], [1173.18017578125, 46.14617919921875], [1164.467529296875, 172.2851104736328], [1299.571044921875, 53.8868408203125], [1290.7852783203125, 179.48536682128906], [1427.1798095703125, 55.33673095703125], [1544.2344970703125, 185.65187072753906], [1417.5098876953125, 182.56861877441406], [535.5181884765625, 268.1347961425781], [659.6305541992188, 274.99664306640625], [783.742919921875, 281.8584899902344], [907.6002807617188, 287.9775085449219], [1032.2950439453125, 293.47186279296875], [1155.7548828125, 298.4240417480469], [1281.99951171875, 305.0838928222656], [1533.680419921875, 314.5171203613281], [1407.8399658203125, 309.8005065917969], [530.0355224609375, 394.3231201171875], [653.6728515625, 400.2030334472656], [777.3101806640625, 406.08294677734375], [901.112060546875, 412.697265625], [1024.970703125, 419.15313720703125], [1149.5826416015625, 425.0092468261719], [1275.947509765625, 431.50457763671875], [1526.871337890625, 441.2618408203125], [1401.409423828125, 436.3832092285156], [523.7191162109375, 515.3825073242188], [647.7335205078125, 522.6635131835938], [771.7479248046875, 529.9445190429688], [895.2698364257812, 536.7974853515625], [1017.65087890625, 543.7203979492188], [1142.193115234375, 548.7980346679688], [1267.1497802734375, 555.5639038085938], [1519.6322021484375, 569.3359985351562], [1393.3909912109375, 562.449951171875], [519.167236328125, 639.2747192382812], [642.6317749023438, 646.0523071289062], [766.0963134765625, 652.8298950195312], [888.9268798828125, 659.2049560546875], [1011.9498901367188, 666.01220703125], [1135.528076171875, 672.4595336914062], [1259.861572265625, 678.7305297851562], [1510.105712890625, 693.2182006835938], [1384.983642578125, 685.974365234375], [514.0538330078125, 760.3330688476562], [637.1481323242188, 767.7413940429688], [760.242431640625, 775.1497192382812], [883.39599609375, 782.2955932617188], [1006.1826171875, 787.8327026367188], [1128.979248046875, 795.0750122070312], [1252.968505859375, 801.9227905273438], [1501.88427734375, 816.1925659179688], [1377.4263916015625, 809.0576782226562], [628.3214721679688, 1011.7009887695312], [510.296875, 881.876220703125], [632.7348022460938, 889.72119140625], [750.10302734375, 1019.9826049804688], [755.1727294921875, 897.566162109375], [868.800537109375, 1028.974609375], [876.0982666015625, 905.6350708007812], [990.63134765625, 1032.544921875], [998.406982421875, 910.1888427734375], [1116.0322265625, 1040.60546875], [1122.5057373046875, 917.8402709960938], [1239.623291015625, 1047.734375], [1246.2958984375, 924.82861328125], [1361.8277587890625, 1058.192138671875], [1492.958251953125, 942.4212646484375], [1369.6270751953125, 933.6249389648438]]


'''