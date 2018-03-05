#ifndef RK4_HPP
#define RK4_HPP



template<unsigned N, void(*F)(double, double const *, double const *,double *)>
class RK4 {
public:
  static void calc(double t, double h, double const *params, double const *pos, double *out) {
    double k1[N], k2[N], k3[N], k4[N], tmp[N];
    F(t, params, pos, k1);
    add(pos, k1, h/2, tmp);
    F(t + h/2, params, tmp, k2);
    add(pos, k2, h/2, tmp);
    F(t + h/2, params, tmp, k3);
    add(pos, k3, h, tmp);
    F(t + h, params, tmp, k4);

    for(unsigned i = 0; i < N; ++i) {
      out[i] = pos[i] + (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) * h/6;
    }
  }
protected:
  static void add(double const *a, double const *b, double scale, double *out) {
    for(unsigned i = 0; i < N; ++i) {
      out[i] = a[i] + scale * b[i];
    }
  }
};

#endif // RK4_HPP
