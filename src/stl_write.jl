
function write_stl(articulated_body::EABM.AbstractArticulatedBody, q::Vector{<:Number}, filename::String)
    state = EABM.StateHarness(q, articulated_body);
    open(filename, "w") do file
        print(file, "solid solid1\n");
        EABM.forward_recurse!( articulated_body, write_articulation_stl, state, file );
        print(file, "endsolid solid1\n");
    end
end

abstract type STLProperties end
mutable struct DesignedSTL <: STLProperties
    stlsolid::Solid
end
mutable struct PrismaticSTL <: STLProperties
    cross_section::Matrix{Float64}
end

function write_articulation_stl(i::EABM.Articulation, s::EABM.StateHarness, f)
    write_articulation_stl(i, i.properties.stl_properties, s, f);
end
function write_articulation_stl(i::EABM.Articulation, p::DesignedSTL, s::EABM.StateHarness, f)
    write_designed_stl(i, p.stlsolid, s, f);
end
function write_articulation_stl(i::EABM.Articulation, p::PrismaticSTL, s::EABM.StateHarness, f)
    write_prismatic_stl(i, s, f);
end

function write_designed_stl(i::EABM.Articulation, solid::Solid, state::EABM.StateHarness, file)
    for f in solid.facets
        writefacet(file, [(state[i].X0[1:3,1:3]'*[v.x, v.y, v.z]) .+ state[i.parent].p for v in f.vertices]...)
    end
end
function write_prismatic_stl(i::EABM.Articulation, s::EABM.StateHarness, f)
    if (i.parent.body_number == 0) || typeof(i.parent.properties.stl_properties) != typeof(i.properties.stl_properties)
        root_vertices = i.properties.stl_properties.cross_section;
        tip_vertices = i.properties.stl_properties.cross_section;

        root_loc = s[i.parent].p;
        tip_loc = s[i].p;

        root_rot = s[i].X0;
        tip_rot = s[i].X0;

        do_root_face = true;
        do_tip_face = true;
        
    elseif typeof(i.parent.properties.stl_properties) == typeof(i.properties.stl_properties)
        root_vertices = i.parent.properties.stl_properties.cross_section;
        tip_vertices = i.properties.stl_properties.cross_section

        root_loc = s[i.parent].p;
        tip_loc = s[i].p;

        root_rot = s[i.parent].X0;
        tip_rot = s[i].X0;

        do_root_face = false;
        do_tip_face = true;
    end

    for c in i.children
        do_tip_face = do_tip_face && !(typeof(c.properties.stl_properties) == typeof(i.properties.stl_properties))
    end

    root_vertices = root_loc .+ (root_rot[1:3,1:3]' * root_vertices);
    tip_vertices = tip_loc .+ (tip_rot[1:3,1:3]' * tip_vertices);

    write_prism_stl(f, root_vertices, tip_vertices, do_root_face, do_tip_face);

end


function write_prism_stl(file, rv, tv, drf, dtf)
    if drf
        for f = 2:size(rv,2)-1
            writefacet(file, rv[:,1],rv[:,f],rv[:,f+1]);
        end
    end

    for df in 1:size(tv,2)
        dfe = df+1
        if dfe > size(tv,2)
            dfe = 1
        end
        writefacet(file,tv[:,df],tv[:,dfe],rv[:,df]);
        writefacet(file,tv[:,dfe],rv[:,df],rv[:,dfe]);
    end
    
    if dtf
        for f = 2:size(tv,2)-1
            writefacet(file, tv[:,1],tv[:,f],tv[:,f+1]);
        end
    end
end



function writefacet(f,v1::Vector{<:Number},v2::Vector{<:Number},v3::Vector{<:Number})
    v1 = vec(v1); v2 = vec(v2); v3 = vec(v3);
    n = (v3-v1)×(v2-v1);
    n = vec(n / norm(n));
    write(f, @sprintf("facet normal %.8f %.8f %.8f\nouter loop\nvertex %.8f %.8f %.8f\nvertex %.8f %.8f %.8f\nvertex %.8f %.8f %.8f\nendloop\nendfacet\n", n[1],n[2],n[3], v1[1], v1[2], v1[3], v2[1], v2[2], v2[3], v3[1], v3[2], v3[3]));
end






