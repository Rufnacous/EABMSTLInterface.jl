module EABMSTLInterface
using LinearAlgebra, Printf, EABM

mutable struct STLVector
    x::Float64
    y::Float64
    z::Float64
end
mutable struct Facet
    vertices::Array{STLVector}
    normal::STLVector
end
mutable struct Solid
    facets::Array{Facet}
end
Solid() = Solid([]);

include("stl_read.jl");
include("stl_write.jl");

mutable struct TestArticulationProperties
    stl_properties::STLProperties
end

function test()
    solid, lvec = import_stl("../robotpart1.stl");
    stlprops = DesignedSTL(solid);
    stl_inertia_about_com, stl_com, mass = calculate_inertia(stlprops.stlsolid);

    a1 = EABM.Articulation(
        1, 1, RotaryJoint(:y),
        nothing, norm(lvec),
        mass, stl_inertia_about_com,
        [1 0 0; 0 1.0 0; 0 0 1], lvec,
        [0.0,0.0,0.0], stl_com,
        TestArticulationProperties(stlprops)
    );
    body = EABM.ArticulatedBody( a1 );
    
    
    solid, lvec = import_stl("../robotpart2.stl");
    stlprops = DesignedSTL(solid);
    stl_inertia, stl_com, mass = calculate_inertia(stlprops.stlsolid);

    a2 = EABM.Articulation(
        2, 2, RotaryJoint(:x),
        nothing, norm(lvec),
        mass, stl_inertia_about_com,
        [1 0 0; 0 1.0 0; 0 0 1], lvec,
        [0.0,0.0,0.0], stl_com,
        TestArticulationProperties(stlprops)
    );
    EABM.join_bodies!(body, a1, a2);
    
    
    solid, lvec = import_stl("../robotpart3.stl");
    stlprops = DesignedSTL(solid);
    stl_inertia, stl_com, mass = calculate_inertia(stlprops.stlsolid);

    a3 = EABM.Articulation(
        3, 3, RotaryJoint(:y),
        nothing, norm(lvec),
        mass, stl_inertia_about_com,
        [1 0 0; 0 1.0 0; 0 0 1], lvec,
        [0.0,0.0,0.0], stl_com,
        TestArticulationProperties(stlprops)
    );
    EABM.join_bodies!(body, a2, a3);

        
    force = force_gravity();
    torque = EABM.torque_damping(c=0.1)

    iq = zeros(body);

    sol = simulate(body, force, torque, 100.0, initcond = zeros(body), integrator=:approx );

    ts = LinRange(0,sol.t[end], 2000)
    for (ti, t) in enumerate(ts)
        write_stl(body, sol(t), @sprintf("outfile_%03d.stl",ti));
    end

end


end
