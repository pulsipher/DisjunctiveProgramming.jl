################################################################################
#                               UTILITY METHODS
################################################################################
# Requred for extensions to MOI.AbstractVectorSet
function _MOI.Utilities.set_dot(x::AbstractVector, y::AbstractVector, set::DisjunctionSet)
    return LinearAlgebra.dot(x, y) # TODO figure out what we should actually do here
end

# TODO create a bridge for `DisjunctionSet`

# TODO create helper method to unpack DisjunctionSet at the MOI side of things

################################################################################
#                            REFRORMULATION METHODS
################################################################################
# Helper methods to handle recursively flattening the disjuncts
function _constr_set!(funcs, con::JuMP.AbstractConstraint)
    append!(funcs, JuMP.jump_function(con))
    return JuMP.moi_set(con)
end
function _constr_set!(funcs, con::Disjunction)
    inner_funcs, set = _disjunction_to_set(con)
    append!(funcs, inner_funcs)
    return set
end

# Create the vectors needed for a disjunction vector constraint
function _disjunction_to_set(d::Disjunction)
    funcs = JuMP.AbstractJuMPScalar[]
    sets = Vector{Vector{_MOI.AbstractSet}}[]
    d_idxs = Int[]
    for lvref in d.indicators
        model = JuMP.owner_model(lvref)
        push!(funcs, _indicator_to_binary(model)[lvref])
        push!(d_idxs, length(funcs))
        crefs = _indicator_to_constraints(model)[lvref]
        push!(sets, [_constr_set!(funcs, JuMP.constraint_object(cref)) for cref in crefs])
    end
    # convert the `sets` type to be concrete if possible (TODO benchmark if this is worth it)
    SetType = typeof(first(sets))
    if SetType != Vector{_MOI.AbstractSet} && all(s -> s isa SetType, sets) 
        sets = convert(SetType, sets)
    end
    return funcs, DisjunctionSet(length(funcs), d_idxs, sets)
end

# Extend the disjunction reformulation
function reformulate_disjunction(
    model::JuMP.Model, 
    d::Disjunction, 
    ::MOIDisjunction
    )
    funcs, set = _disjunction_to_set(d)
    return [JuMP.VectorConstraint(funcs, set, JuMP.VectorShape())]
end
