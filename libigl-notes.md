V = Matrix of vertices ( number of verts x 3 )
F = List of Faces (number of faces x 3 -> for triangular) or 4 for tetmesh
C = List of verts for bones/skeleton (number of verts in bone structure x 3)
BE = List of edges -> indexed with respect to C (number of edges x 2)
P = list of parent index for each edges (row i represent parent for edge i, root bone has -1 parent)
W = weights/impact of bone/edges. row i represent impact of edge j to vertex i. in LBS, a vertex at max has 4 influencing edges so W is a (num of vertex x 4) matrix.
