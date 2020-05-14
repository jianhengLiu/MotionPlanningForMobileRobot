# Motion Planning

# 1 Introduction

**function**:

1. Estimate
   - low latency
   - high accuracy and consistency
2. Perception
3. planning
4. control



**Lattice：栅格**

**Kinodynamic：动态约束**



## Map

| **grid_map栅格地图**   | **https://github.com/ANYbotics/grid_map**      |
| ---------------------- | ---------------------------------------------- |
| **octo_map八叉树地图** | **http://octomap.github.io/**                  |
| **Voxel hashing**      | **https://github.com/niessner/VoxelHashing**   |
| **Point cloud map**    | **https://pointcloud.org**                     |
| TSDF map               | https://github.com/personalrobotics/OpenChisel |



### grid_map栅格地图 

###  https://github.com/ANYbotics/grid_map

查询方法：xyz点



### **octo_map八叉树地图** 

###  http://octomap.github.io/

查询方法：树递归



### **Voxel hashing**

https://github.com/niessner/VoxelHashing

查询方法：字典书签（哈希地址）



### **Point cloud map**

**  **https://pointcloud.org**

查询方法：遍历



### TSDF map

Truncated（截断的） Signed Distance Functions

距离场：障碍物外为正值，内为负值（与距离有正比关系）



### ESDF map

Euclidean（欧式） Signed Distance Functions

距离场：障碍物外为正值，内为负值（与距离有正比关系）



# 2 Search-based Method

## 2.1 General grpah search

图：节点和边（有向/无向）

## 2.1.1 Dijistra

expanded in all directions->high cost

## 2.1.1 A*

**heuristic:启发式的**



mainly toward the goal->lower cost

1. 建立状态空间模型
2. 每次只保存一个状态



## 2.2 Sampling-based Method采样法

### 2.2.1 PRM(Probabilistic Roadmap)



### 2.2.2 RRT*(Random Rapid Tree star)



### 2.2.3 Informed RRT*(启发式RRTstar)

## 

