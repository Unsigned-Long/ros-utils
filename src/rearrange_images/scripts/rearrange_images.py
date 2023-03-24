from genericpath import isdir
import os
import shutil
import matplotlib.pyplot as plt
import numpy as np
import math
import plt_utils as pltu
import shutil

base_dir = '/home/csl/dataset/HCIL-Calib/data_202332320276'
image_hz_inv = 0.1

new_image_dir = 'image_rearrange'
image_dir_stamp_list = {
    'cam_gs': ('stamp_cam_gs.txt', 'image/cam_gs'),
    'cam_rs': ('stamp_cam_rs.txt', 'image/cam_rs')
}


def read_data(filename):
    file = open(filename, "r")
    lines = file.readlines()
    data = []
    drop_frame = []
    idx = 0
    for line in lines:
        if line.find('#DROPFRAME') != -1:
            # use last image's gps time, without image name
            data.append([idx, data[idx - 1][1], None])
            drop_frame.append(idx)
        else:
            item = line.split(',')
            image_name = item[1]
            gps_time = float(item[2])
            data.append([idx, gps_time, image_name])
        idx += 1
    return (data, drop_frame)


if __name__ == '__main__':
    if os.path.exists(base_dir + '/' + new_image_dir):
        shutil.rmtree(base_dir + '/' + new_image_dir)
    for key, stamp_dir in image_dir_stamp_list.items():
        (data, drop_frame) = read_data(base_dir + '/' + stamp_dir[0])
        # drawer
        pltu.drawer.set_fig_size(10.0, 8.0)
        fig, ax = plt.subplots(1, 1)
        ax.plot([elem[0] for elem in data], [elem[1] for elem in data], c='b',
                marker='.', mec='black', mfc='r', ms=15, label='src data timestamps')
        for drop_frame_idx in drop_frame:
            ax.axvline(drop_frame_idx, color='g', linestyle='dashed', linewidth=2,
                       label='drop frame index [' + str(drop_frame_idx) + ']')
        ax.set_title('Use data before \'dropped frames\' to recover timestamps')
        ax.legend()
        pltu.drawer.show_figure()

        # get info
        print('Choose two indexes to model a line, which would used to recover timestamps:')
        fir_idx = input("First index: ")
        fir_gps = data[int(fir_idx)][1]
        sed_idx = input("Sed index: ")
        sed_gps = data[int(sed_idx)][1]
        delta_timestamp = (sed_gps - fir_gps) / (int(sed_idx) - int(fir_idx))
        print('First  idx: ', fir_idx, ', timestamp: ', fir_gps, '; Second idx: ', sed_idx, ', timestamp: ', sed_gps)
        print('delta timestamp: ', delta_timestamp, ', start to recover timestamps...')
        if math.fabs(delta_timestamp - image_hz_inv) > 1E-8:
            print('You chose wrong idx!!!')
            break

        # compute new timestamps
        ref_idx = int(fir_idx)
        ref_timestamp = fir_gps
        dst_data = []
        for i in range(len(data)):
            dist_to_fir_chose = i - ref_idx
            delta_timestamp = dist_to_fir_chose * image_hz_inv
            dst_timestamp = ref_timestamp + delta_timestamp
            dst_data.append([data[i][0], dst_timestamp, str(int(dst_timestamp * 1E5)) + '.jpg'])

        # drawer
        pltu.drawer.set_fig_size(10.0, 8.0)
        fig, ax = plt.subplots(1, 1)
        ax.plot([elem[0] for elem in data], [elem[1] for elem in data], c='b',
                marker='.', mec='black', mfc='r', ms=15, label='src data timestamps')
        ax.plot([elem[0] for elem in dst_data], [elem[1] for elem in dst_data], c='g',
                marker='.', mec='black', mfc='g', ms=15, label='dst data timestamps')
        for drop_frame_idx in drop_frame:
            ax.axvline(drop_frame_idx, color='g', linestyle='dashed', linewidth=2,
                       label='drop frame index [' + str(drop_frame_idx) + ']')
        ax.set_title('Use data before \'dropped frames\' to recover timestamps')
        ax.legend()
        pltu.drawer.show_figure()

        # save
        chose = input("Accept (1) or refuse (0): ")
        if chose == '1':
            os.makedirs(base_dir + '/' + new_image_dir + '/' + key)
            with open(base_dir + '/' + 'stamp_' + key + '_new.txt', "w") as file:
                for i in range(len(dst_data)):
                    if i in drop_frame:
                        continue
                    shutil.copy(base_dir + '/' + stamp_dir[1] + '/' + data[i][2],
                                base_dir + '/' + new_image_dir + '/' + key + '/' + dst_data[i][2])
                    file.write('{:.5f}'.format(dst_data[i][1]) + ',' + dst_data[i][2] + '\n')
