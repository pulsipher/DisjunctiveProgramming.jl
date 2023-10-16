################################################################################
#                              INDICATOR REFORMULATION
################################################################################
#scalar disjunct constraint
function reformulate_disjunct_constraint(
    model::Model,
    con::ScalarConstraint{T, S},
    bvref::VariableRef,
    method::Indicator
) where {T, S}
    reform_con = build_constraint(error, [1*bvref, con.func], _MOI.Indicator{_MOI.ACTIVATE_ON_ONE}(con.set))
    return [reform_con]
end
#vectorized disjunct constraint
function reformulate_disjunct_constraint(
    model::Model,
    con::VectorConstraint{T, S},
    bvref::VariableRef,
    method::Indicator
) where {T, S}
    set = _vec_to_scalar_set(con.set)
    return [
        build_constraint(error, [1*bvref, f], _MOI.Indicator{_MOI.ACTIVATE_ON_ONE}(set)) 
        for f in con.func
    ]
end
#nested indicator reformulation. NOTE: the user needs to provide the appropriate linking constraint for the logical variables for this to work (e.g. w in Exactly(y[1]) to link the parent disjunct y[1] to the nested disjunction w)
function reformulate_disjunct_constraint(
    model::Model,
    con::VectorConstraint{T, S},
    bvref::VariableRef,
    method::Indicator
) where {T, S <: _MOI.Indicator}
    return [con]
end