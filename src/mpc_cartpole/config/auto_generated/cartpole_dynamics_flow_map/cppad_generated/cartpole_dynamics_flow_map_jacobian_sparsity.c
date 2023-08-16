void cartpole_dynamics_flow_map_jacobian_sparsity(unsigned long const** row,
                                                  unsigned long const** col,
                                                  unsigned long* nnz) {
   static unsigned long const rows[8] = {0,1,2,2,2,3,3,3};
   static unsigned long const cols[8] = {3,4,1,3,5,1,3,5};
   *row = rows;
   *col = cols;
   *nnz = 8;
}
