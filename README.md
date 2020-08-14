# Implementation of [Deformation Transfer for Triangle Meshes](http://people.csail.mit.edu/sumner/research/deftransfer/)

Implemented in C++ using CGAL and Eigen. 

## Project Organization
The project consists of two parts:

##### Correspondence Tool
A tool for specifying initial vertex correspondences between the source and target meshes.

##### Transfer
A tool to transfer the deformation of the source mesh to the target mesh using the initial correspondences specified in the correspondence tool.
 
## Usage
##### Correspondence Tool
````
./corrtool [source_mesh] [target_mesh] [ouput_path]
````
- source_mesh - Path to the source mesh
- target_mesh - Path to the target mesh
- output_path - Path to the output file. If it already exists, existing correspondences will be loaded.

##### Transfer
````
./transfer [source_ref_mesh] [source_deform_mesh] [target_ref_mesh] [vert_corr] [ouput_path]
````
- source_ref_mesh - Path to the reference source mesh
- source_deform_mesh - Path to the deformed source mesh
- target_ref_mesh - Path to the reference target mesh
- vert_corr - Path to the vertex correspondence
- output_path - Path to the output file