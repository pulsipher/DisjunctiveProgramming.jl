################################################################################
#                              LOGICAL VARIABLES
################################################################################
"""

Create binary (indicator) variables for logic variables.
"""
function _reformulate_logical_variables(model::JuMP.Model)
    for (lv_idx, lv_data) in _logical_variables(model)
        lv = lv_data.variable
        var = JuMP.ScalarVariable(_variable_info(binary=true, start_value = lv.start_value, fix_value = lv.fix_value))
        bvref = JuMP.add_variable(model, var, lv_data.name)
        _indicator_to_binary(model)[lv_idx] = JuMP.index(bvref)
    end
end

################################################################################
#                              DISJUNCTIONS
################################################################################
# disjunctions
function _reformulate_disjunctions(model::JuMP.Model, method::AbstractReformulationMethod)
    for (disj_idx, disj) in _disjunctions(model)
        _reformulate_disjuncts(model, disj_idx, disj, method)
    end
end

function _reformulate_disjuncts(model::JuMP.Model, disj_idx::DisjunctionIndex, disj::ConstraintData{T}, method::Union{BigM,Indicator}) where {T<:Disjunction}
    for d in disj.constraint.disjuncts
        _reformulate_disjunct(model, d, method)
    end
end

function _reformulate_disjuncts(model::JuMP.Model, disj_idx::DisjunctionIndex, disj::ConstraintData{T}, method::Hull) where {T<:Disjunction}
    disj_vrefs = _get_disjunction_variables(disj)
    _update_variable_bounds.(disj_vrefs) 
    for d in disj.constraint.disjuncts #reformulate each disjunct
        _disaggregate_variables(model, d, disj_vrefs) #disaggregate variables for that disjunct
        _reformulate_disjunct(model, d, method)
    end
    for vref in disj_vrefs #create sum constraint for disaggregated variables
        _aggregate_variable(model, vref, disj_idx)
    end
end

# individual disjuncts
function _reformulate_disjunct(model::JuMP.Model, d::Disjunct, method::AbstractReformulationMethod)
    #reformulate each constraint and add to the model
    ind_idx = JuMP.index(d.indicator) #logical indicator
    bv_idx = _indicator_to_binary(model)[ind_idx]
    bvref = JuMP.VariableRef(model, bv_idx)
    for con in d.constraints
        cdata = _disjunct_constraints(model)[JuMP.index(con)]
        _reformulate_disjunct_constraint(model, cdata.constraint, bvref, method)
    end
end

# BigM: individual disjunct constraints
function _reformulate_disjunct_constraint(
    model::JuMP.Model,
    con::JuMP.ScalarConstraint{T, S}, 
    bvref::JuMP.VariableRef,
    method::BigM
) where {T, S <: _MOI.LessThan}
    #TODO: need to pass _error to build_constraint
    M = _get_M_value(method, con.func, con.set)
    new_func = JuMP.@expression(model, con.func - M*(1-bvref))
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, new_func, con.set)
    )
end
function _reformulate_disjunct_constraint(
    model::JuMP.Model,
    con::JuMP.VectorConstraint{T, S}, 
    bvref::JuMP.VariableRef,
    method::BigM
) where {T, S <: _MOI.Nonpositives}
    #TODO: need to pass _error to build_constraint
    M = JuMP.@expression(model, [i=1:con.set.dimension],
        _get_M_value(method, con.func[i], con.set)
    )
    new_func = JuMP.@expression(model, [i=1:con.set.dimension], 
        con.func[i] - M[i]*(1-bvref)
    )
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, new_func, con.set)
    )
end
function _reformulate_disjunct_constraint(
    model::JuMP.Model, 
    con::JuMP.ScalarConstraint{T, S}, 
    bvref::JuMP.VariableRef,
    method::BigM,
) where {T, S <: _MOI.GreaterThan}
    #TODO: need to pass _error to build_constraint
    M = _get_M_value(method, con.func, con.set)
    new_func = JuMP.@expression(model, con.func + M*(1-bvref))
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, new_func, con.set)
    )
end
function _reformulate_disjunct_constraint(
    model::JuMP.Model, 
    con::JuMP.VectorConstraint{T, S}, 
    bvref::JuMP.VariableRef,
    method::BigM,
) where {T, S <: _MOI.Nonnegatives}
    #TODO: need to pass _error to build_constraint
    M = JuMP.@expression(model, [i=1:con.set.dimension],
        _get_M_value(method, con.func[i], con.set)
    )
    new_func = JuMP.@expression(model, [i=1:con.set.dimension], 
        con.func[i] + M[i]*(1-bvref)
    )
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, new_func, con.set)
    )
end
function _reformulate_disjunct_constraint(
    model::JuMP.Model, 
    con::JuMP.ScalarConstraint{T, S}, 
    bvref::JuMP.VariableRef,
    method::BigM
) where {T, S <: _MOI.Interval}
    #TODO: need to pass _error to build_constraint
    M = _get_M_value(method, con.func, con.set)
    new_func_gt = JuMP.@expression(model, con.func + M[1]*(1-bvref))
    new_func_lt = JuMP.@expression(model, con.func - M[2]*(1-bvref))
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, new_func_gt, _MOI.GreaterThan(con.set.lower))
    )
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, new_func_lt, _MOI.LessThan(con.set.upper))
    )
end
function _reformulate_disjunct_constraint(
    model::JuMP.Model, 
    con::JuMP.ScalarConstraint{T, S}, 
    bvref::JuMP.VariableRef,
    method::BigM
) where {T, S <: _MOI.EqualTo}
    #TODO: need to pass _error to build_constraint
    M = _get_M_value(method, con.func, con.set)
    new_func_gt = JuMP.@expression(model, con.func + M[1]*(1-bvref))
    new_func_lt = JuMP.@expression(model, con.func - M[2]*(1-bvref))
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, new_func_gt, _MOI.GreaterThan(con.set.value))
    )
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, new_func_lt, _MOI.LessThan(con.set.value))
    )
end
function _reformulate_disjunct_constraint(
    model::JuMP.Model, 
    con::JuMP.VectorConstraint{T, S}, 
    bvref::JuMP.VariableRef,
    method::BigM
) where {T, S <: _MOI.Zeros}
    #TODO: need to pass _error to build_constraint
    M = JuMP.@expression(model, [i=1:con.set.dimension],
        _get_M_value(method, con.func[i], con.set)
    )
    new_func_nn = JuMP.@expression(model, [i=1:con.set.dimension], 
        con.func[i] + M[i][1]*(1-bvref)
    )
    new_func_np = JuMP.@expression(model, [i=1:con.set.dimension], 
        con.func[i] - M[i][2]*(1-bvref)
    )
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, new_func_nn, _MOI.Nonnegatives(con.set.dimension))
    )
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, new_func_np, _MOI.Nonpositives(con.set.dimension))
    )
end

# Hull: individual disjunct constraints
function _reformulate_disjunct_constraint(
    model::JuMP.Model, 
    con::JuMP.ScalarConstraint{T, S}, 
    bvref::JuMP.VariableRef,
    method::Hull
) where {T, S}
    #TODO: need to pass _error to build_constraint
    new_con = _disaggregate_constraint(model, con, bvref, method)
    JuMP.add_constraint(model, new_con)
end
function _reformulate_disjunct_constraint(
    model::JuMP.Model, 
    con::JuMP.ScalarConstraint{T, S}, 
    bvref::JuMP.VariableRef,
    method::Hull
) where {T, S <: _MOI.Interval}
    #TODO: need to pass _error to build_constraint
    new_con_gt, new_con_lt = _disaggregate_constraint(model, con, bvref, method)
    JuMP.add_constraint(model, new_con_gt)
    JuMP.add_constraint(model, new_con_lt)
end

# Indicator: individual disjunct constraints
function _reformulate_disjunct_constraint(
    model::JuMP.Model,
    con::JuMP.ScalarConstraint{JuMP.AffExpr, S},
    bvref::JuMP.VariableRef,
    ::Indicator
) where {S}
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, [1*bvref, con.func], _MOI.Indicator{_MOI.ACTIVATE_ON_ONE}(con.set))
    )
end

# define fallbacks for other constraint types
function _reformulate_disjunct_constraint(
    ::JuMP.Model,  
    con::JuMP.AbstractConstraint, 
    ::JuMP.VariableRef,
    method::AbstractReformulationMethod
)
    error("$method reformulation for constraint $con is not supported yet.")
end

################################################################################
#                              LOGICAL CONSTRAINTS
################################################################################

"""

"""
function _reformulate_logical_constraints(model::JuMP.Model)
    for (_, lcon) in _logical_constraints(model)
        _reformulate_logical_constraint(model, lcon.constraint.func, lcon.constraint.set)
    end
end

function _reformulate_logical_constraint(model::JuMP.Model, lvec::Vector{LogicalVariableRef}, set::Union{MOIAtMost, MOIAtLeast, MOIExactly})
    return _reformulate_selector(model, set, set.value, lvec)
end

function _reformulate_logical_constraint(model::JuMP.Model, lexpr::_LogicalExpr, ::_MOI.EqualTo{Bool})
    return _reformulate_proposition(model, lexpr)
end

function _reformulate_selector(model::JuMP.Model, ::MOIAtLeast, val::Number, lvrefs::Vector{LogicalVariableRef})
    bvrefs = _indicator_to_binary_ref.(lvrefs)
    return JuMP.add_constraint(model,
        JuMP.build_constraint(error, JuMP.@expression(model, sum(bvrefs)), _MOI.GreaterThan(val))
    )
end
function _reformulate_selector(model::JuMP.Model, ::MOIAtMost, val::Number, lvrefs::Vector{LogicalVariableRef})
    bvrefs = _indicator_to_binary_ref.(lvrefs)
    return JuMP.add_constraint(model,
        JuMP.build_constraint(error, JuMP.@expression(model, sum(bvrefs)), _MOI.LessThan(val))
    )
end
function _reformulate_selector(model::JuMP.Model, ::MOIExactly, val::Number, lvrefs::Vector{LogicalVariableRef})
    bvrefs = _indicator_to_binary_ref.(lvrefs)
    return JuMP.add_constraint(model,
        JuMP.build_constraint(error, JuMP.@expression(model, sum(bvrefs)), _MOI.EqualTo(val))
    )
end
function _reformulate_selector(model::JuMP.Model, ::MOIAtLeast, lvref::LogicalVariableRef, lvrefs::Vector{LogicalVariableRef})
    bvref = _indicator_to_binary_ref(lvref)
    bvrefs = Vector{Any}(_indicator_to_binary_ref.(lvrefs))
    return JuMP.add_constraint(model,
        build_constraint(error, JuMP.@expression(model, sum(bvrefs) - bvref), _MOI.GreaterThan(0))
    )
end
function _reformulate_selector(model::JuMP.Model, ::MOIAtMost, lvref::LogicalVariableRef, lvrefs::Vector{LogicalVariableRef})
    bvref = _indicator_to_binary_ref(lvref)
    bvrefs = Vector{Any}(_indicator_to_binary_ref.(lvrefs))
    return JuMP.add_constraint(model,
        build_constraint(error, JuMP.@expression(model, sum(bvrefs) - bvref), _MOI.LessThan(0))
    )
end
function _reformulate_selector(model::JuMP.Model, ::MOIExactly, lvref::LogicalVariableRef, lvrefs::Vector{LogicalVariableRef})
    bvref = _indicator_to_binary_ref(lvref)
    bvrefs = Vector{Any}(_indicator_to_binary_ref.(lvrefs))
    return JuMP.add_constraint(model,
        build_constraint(error, JuMP.@expression(model, sum(bvrefs) - bvref), _MOI.EqualTo(0))
    )
end

function _reformulate_proposition(model::JuMP.Model, lexpr::_LogicalExpr)
    expr = _to_cnf(lexpr)
    if expr.head == :∧
        for arg in expr.args
            _add_proposition(model, arg)
        end
    elseif expr.head == :∨ && all(_isa_literal.(expr.args))
        _add_proposition(model, expr)
    else
        error("Expression was not converted to proper Conjunctive Normal Form:\n$expr")
    end
end

_isa_literal(v::LogicalVariableRef) = true
_isa_literal(v::_LogicalExpr) = (v.head == :¬) && (length(v.args) == 1) && _isa_literal(v.args[1])
_isa_literal(v) = false

function _add_proposition(model::JuMP.Model, arg::Union{LogicalVariableRef,_LogicalExpr})
    func = _reformulate_clause(model, arg)
    if !isempty(func.terms) && !all(iszero.(values(func.terms)))
        con = JuMP.build_constraint(error, func, _MOI.GreaterThan(1))
        JuMP.add_constraint(model, con)
    end
    return
end

function _reformulate_clause(model::JuMP.Model, lvref::LogicalVariableRef)
    func = 1 * _indicator_to_binary_ref(lvref)
    return func
end

function _reformulate_clause(model::JuMP.Model, lexpr::_LogicalExpr)
    func = JuMP.AffExpr() #initialize func expression
    if _isa_literal(lexpr)
        func += (1 - _reformulate_clause(model, lexpr.args[1]))
    elseif lexpr.head == :∨
        for literal in lexpr.args
            if literal isa LogicalVariableRef
                func += _reformulate_clause(model, literal)
            elseif _isa_literal(literal)
                func += (1 - _reformulate_clause(model, literal.args[1]))
            else
                error("Expression was not converted to proper Conjunctive Normal Form:\n$literal is not a literal.")
            end
        end
    else
        error("Expression was not converted to proper Conjunctive Normal Form:\n$lexpr.")
    end
    
    return func
end