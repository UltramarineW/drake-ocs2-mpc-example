void cppad_cg_models(char const *const** names,
                     int* count) {
   static const char* const models[] = {
      "cartpole_dynamics_guard_surfaces"};
   *names = models;
   *count = 1;
}

