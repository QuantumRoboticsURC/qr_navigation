#!/usr/bin/env python3

"""	José Ángel del Ángel
    joseangeldelangel10@gmail.com

Modified (15/12/2022): 

Code description:
    THIS IS NOT A ROS NODE, this is a script to test the logic of the snail trayectory generator embedded
    into the navigation controller node, feel free to run this script and see how this logic generates
    and snail trayectory with any number of turns and origin you specify :)

Notes:

"""

import matplotlib.pyplot as plt

def create_snail_cords(origin = (0,0), num_turns = 2):
    cords_list = [origin]
    current_cord = origin
    current_turn = 1
    disatnce_between_each_sanil_point = 5
    while current_cord != (origin[0] + num_turns*disatnce_between_each_sanil_point, origin[1] -num_turns*disatnce_between_each_sanil_point):
        
        if current_cord == (origin[0] + current_turn*disatnce_between_each_sanil_point, origin[1] -current_turn*disatnce_between_each_sanil_point):
            current_turn += 1
        
        if  (current_cord[1] - disatnce_between_each_sanil_point >= origin[1] -current_turn*disatnce_between_each_sanil_point and
        (current_cord[0], current_cord[1] - disatnce_between_each_sanil_point) not in cords_list ):
            current_cord = (current_cord[0], current_cord[1] - disatnce_between_each_sanil_point)
            cords_list.append(current_cord)
        elif (current_cord[0] - disatnce_between_each_sanil_point >= origin[0] -current_turn*disatnce_between_each_sanil_point and
        (current_cord[0] - disatnce_between_each_sanil_point, current_cord[1]) not in cords_list ):
            current_cord = (current_cord[0] - disatnce_between_each_sanil_point, current_cord[1])
            cords_list.append(current_cord)
        elif (current_cord[1] + disatnce_between_each_sanil_point <= origin[1] + current_turn*disatnce_between_each_sanil_point and
        (current_cord[0], current_cord[1] + disatnce_between_each_sanil_point) not in cords_list ):
            current_cord = (current_cord[0], current_cord[1] + disatnce_between_each_sanil_point)
            cords_list.append(current_cord)
        elif (current_cord[0] + disatnce_between_each_sanil_point <= origin[0] + disatnce_between_each_sanil_point*current_turn and
        (current_cord[0] + disatnce_between_each_sanil_point, current_cord[1]) not in cords_list ):
            current_cord = (current_cord[0] + disatnce_between_each_sanil_point, current_cord[1])
            cords_list.append(current_cord)

    print(cords_list)
    return cords_list

if __name__ == "__main__":
    result = create_snail_cords()
    x_y = list(zip(*result))
    plt.plot(x_y[0], x_y[1])
    plt.show()