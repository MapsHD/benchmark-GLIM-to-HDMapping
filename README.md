# GLIM to HDMapping simlified instruction

## Step 1 (prepare data)
Download the dataset `kitti_seq00_ros2.zip` by clicking [link](https://huggingface.co/datasets/kubchud/kitti_to_ros/resolve/main/kitti_seq00_ros2.zip) (it is part of [kitti_seq](https://github.com/Jakubach/kitti_to_ros)).

### Extract the dataset

Folder `kitti_seq00_ros2.zip`.

```shell
unzip kitti_seq00_ros2.zip
```
After extraction, the folder name will be `kitti_seq00_ros2` is an input for further calculations. (without the `.zip` extension).

It should be located in `~/hdmapping-benchmark/data`. 

## Step 2 (prepare docker)

```shell
mkdir -p ~/hdmapping-benchmark
cd ~/hdmapping-benchmark
git clone https://github.com/MapsHD/benchmark-GLIM-to-HDMapping.git --recursive
cd benchmark-GLIM-to-HDMapping
git checkout kitti
docker build -t glim_humble .
```

## Step 3 (run docker, file 'kitti_seq00_ros2' should be in '~/hdmapping-benchmark/data')
```shell
cd ~/hdmapping-benchmark/benchmark-GLIM-to-HDMapping
chmod +x docker_session_run-ros2-glim.sh 
cd ~/hdmapping-benchmark/data
~/hdmapping-benchmark/benchmark-GLIM-to-HDMapping/docker_session_run-ros2-glim.sh kitti_seq00_ros2/2011_10_03_drive_0027_extract_ros2 .
```

## Step 4 (Open and visualize data)
Expected data should appear in ~/hdmapping-benchmark/data/output_hdmapping-glim
Use tool [multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping) to open session.json from ~/hdmapping-benchmark/data/output_hdmapping-glim.

You should see following data

lio_initial_poses.reg

poses.reg

scan_lio_*.laz

session.json

trajectory_lio_*.csv

## Contact email
januszbedkowski@gmail.com