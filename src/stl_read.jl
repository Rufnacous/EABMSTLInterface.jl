
function import_stl(stlfile)
    solid, com = open(stlfile) do f
        solid = Solid();
        com = parse_stl_line(solid, readlines(f))
        return solid, com;
    end
    println("Solid with ",length(solid.facets), " facets loaded.")
    return solid, com;
end

function parse_stl_line(solid::Solid, lines::Vector{String})
    com = [0,0,0];
    for newline in lines
        if newline[1] == 'v'
            push!(solid.facets[end].vertices, STLVector(parse.(Float64, split(newline, " ")[2:end])...));
        elseif newline[1] == 'f'
            push!(solid.facets, Facet([], STLVector(parse.(Float64, split(newline, " ")[3:end])...)))
        elseif newline[1] == 's'
            com = parse.(Float64, split(newline, " ")[2:end])
        end
    end
    return com
end

function calculate_inertia(solid::Solid; n_rand=1000)
    mincoords = [Inf, Inf, Inf]
    maxcoords = [-Inf, -Inf, -Inf]

    for f in solid.facets
        for v in f.vertices
            mincoords[1] = min(mincoords[1], v.x);
            mincoords[2] = min(mincoords[2], v.y);
            mincoords[3] = min(mincoords[3], v.z);
            
            maxcoords[1] = max(maxcoords[1], v.x);
            maxcoords[2] = max(maxcoords[2], v.y);
            maxcoords[3] = max(maxcoords[3], v.z);
        end
    end
    randcoords = mincoords .+ (maxcoords .- mincoords) .* rand(3, n_rand);
    
    com = zeros(3);
    inside = zeros(Bool, n_rand)

    for i in 1:n_rand
        if randcoords[:,i] in solid
            inside[i] = true;
            x, y, z = randcoords[:, i]
            com .+= [x, y, z]
            
        end
    end
    num_inside = sum(inside)
    if num_inside > 0
        com /= num_inside
    end

    J = zeros(3,3);

    for i in 1:n_rand
        if inside[i]
            x, y, z = randcoords[:, i] .- com
            
            # Diagonal terms
            J[1,1] += y^2 + z^2
            J[2,2] += x^2 + z^2
            J[3,3] += x^2 + y^2

            # Off-diagonal terms (negative because products of inertia)
            J[1,2] -= x * y
            J[1,3] -= x * z
            J[2,3] -= y * z
        end
    end
    J[2,1] = J[1,2]
    J[3,1] = J[1,3]
    J[3,2] = J[2,3]
    
    if num_inside > 0
        J /= num_inside
    end


    return J, com, 1
end

function ray_intersects_facet(point::Vector{Float64}, facet::Facet)
    v0, v1, v2 = facet.vertices
    
    # Convert vertices to vectors
    v0 = [v0.x, v0.y, v0.z]
    v1 = [v1.x, v1.y, v1.z]
    v2 = [v2.x, v2.y, v2.z]
    
    # Define a ray along the +z direction
    ray_dir = [0.0, 0.0, 1.0]
    
    # Edge vectors
    e1 = v1 - v0
    e2 = v2 - v0
    h = cross(ray_dir, e2)
    a = dot(e1, h)
    
    if abs(a) < 1e-6  # Parallel ray and triangle
        return false
    end
    
    f = 1.0 / a
    s = point - v0
    u = f * dot(s, h)
    if u < 0.0 || u > 1.0
        return false
    end
    
    q = cross(s, e1)
    v = f * dot(ray_dir, q)
    if v < 0.0 || (u + v) > 1.0
        return false
    end
    
    t = f * dot(e2, q)
    return t > 1e-6  # Ensure intersection is in the positive ray direction
end

function Base.in(point::Vector{Float64}, solid::Solid)
    intersections = 0
    for facet in solid.facets
        if ray_intersects_facet(point, facet)
            intersections += 1
        end
    end
    return intersections % 2 == 1  # Odd intersections mean inside
end