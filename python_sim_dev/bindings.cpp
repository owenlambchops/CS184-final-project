// bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>        // automatic std::vector / std::map conversion
#include <pybind11/numpy.h>      // numpy array support

#include  "water_sim_basic.cpp"             // your existing header with all the structs/functions

namespace py = pybind11;


// ── Converters between numpy arrays and std::vector<Vec3> ──────────────────
// The notebook works with numpy arrays of shape (N,3), so we convert at the
// boundary rather than forcing the notebook to use custom C++ types.

std::vector<Vec3> numpy_to_vec3(const py::array_t<float>& arr) {
    auto r = arr.unchecked<2>();
    if (r.shape(1) != 3)
        throw std::runtime_error("Expected array of shape (N, 3)");
    std::vector<Vec3> out(r.shape(0));
    for (ssize_t i = 0; i < r.shape(0); ++i)
        out[i] = {r(i,0), r(i,1), r(i,2)};
    return out;
}

py::array_t<float> vec3_to_numpy(const std::vector<Vec3>& vecs) {
    py::array_t<float> out({(ssize_t)vecs.size(), (ssize_t)3});
    auto r = out.mutable_unchecked<2>();
    for (ssize_t i = 0; i < (ssize_t)vecs.size(); ++i) {
        r(i,0) = vecs[i].x;
        r(i,1) = vecs[i].y;
        r(i,2) = vecs[i].z;
    }
    return out;
}


PYBIND11_MODULE(droplet_cpp, m) {
    m.doc() = "Water droplet simulation — C++ backend";


    // ── SimParams ────────────────────────────────────────────────────────────
    // Exposed as a plain Python class with readable/writable attributes so the
    // notebook can do:  sp = droplet_cpp.SimParams(); sp.gamma = 0.8
    py::class_<SimParams>(m, "SimParams")
        .def(py::init<>())
        .def_property("g",
            [](const SimParams& sp) {
                return std::array<float,3>{sp.g.x, sp.g.y, sp.g.z};
            },
            [](SimParams& sp, std::array<float,3> v) {
                sp.g = Vec3{v[0], v[1], v[2]};
            })
        .def_readwrite("gamma",             &SimParams::gamma)
        .def_readwrite("mu",                &SimParams::mu)
        .def_readwrite("eta",               &SimParams::eta)
        .def_readwrite("k_v",               &SimParams::k_v)
        .def_readwrite("density",           &SimParams::density)
        .def_readwrite("rho",               &SimParams::rho)
        .def_readwrite("boundary_alpha",    &SimParams::boundary_alpha)
        .def_readwrite("alpha",             &SimParams::alpha)
        .def_readwrite("friction_coeff",    &SimParams::friction_coeff)
        .def_readwrite("receding_angle",    &SimParams::receding_angle)
        .def_readwrite("advancing_angle",   &SimParams::advancing_angle)
        .def_readwrite("adhesion_dist",     &SimParams::adhesion_dist)
        .def_readwrite("max_internal_accel",&SimParams::max_internal_accel)
        .def_readwrite("damping_gain",      &SimParams::damping_gain);


    // ── generate_polyhedron ──────────────────────────────────────────────────
    // Returns a tuple: (vertices, faces, neighbours)
    //   vertices  : np.ndarray shape (N, 3) float32
    //   faces     : list of [i0, i1, i2] lists
    //   neighbours: dict {int: [int, ...]}
    m.def("generate_polyhedron",
        [](int n_vertices, float radius) -> py::tuple {
            DropletMesh poly = generate_polyhedron(n_vertices, radius);

            py::array_t<float> verts = vec3_to_numpy(poly.vertices);

            // faces → list[list[int]]
            py::list faces_py;
            for (const Face& f : poly.faces) {
                py::list tri;
                tri.append(f[0]); tri.append(f[1]); tri.append(f[2]);
                faces_py.append(tri);
            }

            // neighbours → dict[int, list[int]]  (pybind11/stl.h handles this)
            py::dict nb_py;
            for (auto& [k, nbs] : poly.neighbours)
                nb_py[py::int_(k)] = nbs;

            return py::make_tuple(verts, faces_py, nb_py);
        },
        py::arg("n_vertices") = 40,
        py::arg("radius")     = 1.0f,
        R"doc(
            Generate a convex polyhedron representing the initial droplet.

            Returns
            -------
            vertices  : np.ndarray, shape (N, 3), float32
            faces     : list of [i0, i1, i2]
            neighbours: dict {vertex_index: [neighbour_indices]}
        )doc");


    // ── compute_vertex_normals_and_volume ────────────────────────────────────
    // Returns a tuple: (normals, volume)
    //   normals : np.ndarray shape (N, 3) float32
    //   volume  : float
    m.def("compute_vertex_normals_and_volume",
        [](const py::array_t<float>& x_np,
           const py::list&           faces_py) -> py::tuple {

            std::vector<Vec3> x = numpy_to_vec3(x_np);

            std::vector<Face> faces;
            for (auto item : faces_py) {
                auto tri = item.cast<std::vector<int>>();
                faces.push_back({tri[0], tri[1], tri[2]});
            }

            auto [normals, volume] = compute_vertex_normals_and_volume(x, faces);
            return py::make_tuple(vec3_to_numpy(normals), volume);
        },
        py::arg("x"),
        py::arg("faces"),
        R"doc(
            Compute per-vertex normals and enclosed volume of the mesh.

            Parameters
            ----------
            x     : np.ndarray, shape (N, 3), float32
            faces : list of [i0, i1, i2]

            Returns
            -------
            normals : np.ndarray, shape (N, 3), float32
            volume  : float
        )doc");


    // ── step_simulation ──────────────────────────────────────────────────────
    // Returns a tuple: (v_new, x_new) both np.ndarray shape (N, 3) float32
    m.def("step_simulation",
        [](const py::array_t<float>& v_np,
           const py::array_t<float>& x_np,
           const py::list&           faces_py,
           const py::dict&           neighbours_py,
           float                     V_0,
           float                     dt,
           const SimParams&          sp) -> py::tuple {

            std::vector<Vec3> v = numpy_to_vec3(v_np);
            std::vector<Vec3> x = numpy_to_vec3(x_np);

            // faces
            std::vector<Face> faces;
            for (auto item : faces_py) {
                auto tri = item.cast<std::vector<int>>();
                faces.push_back({tri[0], tri[1], tri[2]});
            }

            // neighbours dict → std::map<int, std::vector<int>>
            std::map<int, std::vector<int>> neighbours;
            for (auto& kv : neighbours_py) {
                int key = kv.first.cast<int>();
                neighbours[key] = kv.second.cast<std::vector<int>>();
            }

            auto [v_new, x_new] = step_simulation(
                v, x, faces, neighbours, V_0, dt, sp);

            return py::make_tuple(
                vec3_to_numpy(v_new),
                vec3_to_numpy(x_new));
        },
        py::arg("v"),
        py::arg("x"),
        py::arg("faces"),
        py::arg("neighbours"),
        py::arg("V_0"),
        py::arg("dt"),
        py::arg("params") = SimParams{},
        R"doc(
            Advance the simulation by one time step.

            Parameters
            ----------
            v          : np.ndarray, shape (N, 3), float32  — velocities
            x          : np.ndarray, shape (N, 3), float32  — positions
            faces      : list of [i0, i1, i2]
            neighbours : dict {int: [int, ...]}
            V_0        : float  — rest volume
            dt         : float  — time step
            params     : SimParams (optional, uses defaults if omitted)

            Returns
            -------
            v_new : np.ndarray, shape (N, 3), float32
            x_new : np.ndarray, shape (N, 3), float32
        )doc");
}