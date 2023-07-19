"""

Create binary (indicator) variables for logic variables.
"""
function _reformulate_logical_variables(model::JuMP.Model)
    ind_var_dict = gdp_data(model).indicator_variables
    for (idx, _) in logical_variables(model)
        lvar = LogicalVariableRef(model, idx)
        bvar = JuMP.@variable(model, 
            base_name = string(lvar), 
            binary = true,
        )
        ind_var_dict[lvar] = bvar
    end
end

"""

"""
function _reformulate_disjunctive_constraints(model::JuMP.Model, method::AbstractReformulationMethod)
    for (_, disj) in disjunctive_constraints(model)
        _reformulate_disjunctive_constraints(model, method, disj)
    end
end

function _reformulate_disjunctive_constraints(model::JuMP.Model, method::Union{BigM,Indicator}, disj::DisjunctiveConstraintData)
    @show ind_var_dict = gdp_data(model).indicator_variables
    for d in disj.constraint.disjuncts
        bvar = ind_var_dict[d.indicator]
        _reformulate_disjunctive_constraints(model, method, d, bvar)
    end
end

function _reformulate_disjunctive_constraints(model::JuMP.Model, method::Hull, disj::DisjunctiveConstraintData)
    ind_var_dict = gdp_data(model).indicator_variables
    var_bounds_dict = gdp_data(model).variable_bounds
    disj_vars = _get_disjunction_variables(disj)
    sum_disag_vars = Dict(var => JuMP.AffExpr() for var in disj_vars) #initialize sum constraint for disaggregated variables
    _update_variable_bounds!(var_bounds_dict, disj_vars) #update variable bounds dict
    #reformulate each disjunct
    for d in disj.constraint.disjuncts
        bvar = ind_var_dict[d.indicator]
        #create disaggregated variables for that disjunct
        for var in disj_vars
            JuMP.is_binary(var) && continue #skip binary variables
            #create disaggregated var
            disag_var = _disaggregate_variable(model, d, var, bvar)
            #update aggregation constraint
            JuMP.add_to_expression!(sum_disag_vars[var], 1, disag_var)
        end
        #reformulate disjunct
        _reformulate_disjunctive_constraints(model, method, d, bvar)
    end
    #create sum constraint for disaggregated variables
    for var in disj_vars
        JuMP.is_binary(var) && continue #skip binary variables
        JuMP.@constraint(model, var == sum_disag_vars[var])
    end
end

"""

"""
function _reformulate_disjunctive_constraints(model::JuMP.Model, method::AbstractReformulationMethod, d::Disjunct, bvar::JuMP.VariableRef)
    #reformulate each constraint and add to the model
    for con in d.constraints
        _reformulate_disjunctive_constraints(model, method, con, bvar)
    end
end

"""

"""
function _reformulate_disjunctive_constraints(model::JuMP.Model, method::AbstractReformulationMethod, con::JuMP.AbstractArray{T}, args...) where {T <: JuMP.AbstractConstraint}
    for c in con
        _reformulate_disjunctive_constraints(model, method, c, args...)
    end
end

"""

"""
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::BigM,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
) where {T, S <: _MOI.LessThan}
    #TODO: need to pass _error to build_constraint
    M = method.tighten ? _calculate_tight_M(con) : method.value
    if isinf(M)
        M = method.value
    end
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con.func - M*(1-bvar), con.set)
    )
end
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::BigM,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
) where {T, S <: _MOI.GreaterThan}
    #TODO: need to pass _error to build_constraint
    M = method.tighten ? _calculate_tight_M(con) : method.value
    if isinf(M)
        M = method.value
    end
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con.func + M*(1-bvar), con.set)
    )
end
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::BigM,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
) where {T, S <: _MOI.Interval}
    #TODO: need to pass _error to build_constraint
    M = method.tighten ? _calculate_tight_M(con) : (method.value, method.value)
    if isinf(first(M))
        M[1] = method.value
    end
    if isinf(last(M))
        M[2] = method.value
    end
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con.func + first(M)*(1-bvar), _MOI.GreaterThan(con.set.lower))
    )
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con.func - last(M)*(1-bvar), _MOI.LessThan(con.set.upper))
    )
end
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::BigM,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
) where {T, S <: _MOI.EqualTo}
    #TODO: need to pass _error to build_constraint
    M = method.tighten ? _calculate_tight_M(con) : (method.value, method.value)
    if isinf(first(M))
        M[1] = method.value
    end
    if isinf(last(M))
        M[2] = method.value
    end
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con.func + first(M)*(1-bvar), _MOI.GreaterThan(con.set.value))
    )
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con.func - last(M)*(1-bvar), _MOI.LessThan(con.set.value))
    )
end

"""

"""
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::Hull,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
) where {T <: Union{JuMP.AffExpr, JuMP.QuadExpr}, S <: _MOI.LessThan}
    #TODO: need to pass _error to build_constraint
    con_func = _disaggregate_expression(model, con.func, bvar, method)
    con_func -= con.set.upper * bvar

    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con_func, _MOI.LessThan(0))
    )
end
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::Hull,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
) where {T <: JuMP.NonlinearExpr, S <: _MOI.LessThan}
    #TODO: need to pass _error to build_constraint
    con_func, con_func0 = _disaggregate_nl_expression(model, con.func, bvar, method)
    ϵ = method.value
    con_func = ((1-ϵ)*bvar+ϵ)*con_func - ϵ*(1-bvar)*con_func0 - con.set.upper*bvar
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con_func, _MOI.LessThan(0))
    )
end
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::Hull,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
) where {T <: Union{JuMP.AffExpr, JuMP.QuadExpr}, S <: _MOI.GreaterThan}
    #TODO: need to pass _error to build_constraint
    con_func = _disaggregate_expression(model, con.func, bvar, method)
    con_func -= con.set.lower * bvar
    
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con_func, _MOI.GreaterThan(0))
    )
end
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::Hull,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
) where {T <: JuMP.NonlinearExpr, S <: _MOI.GreaterThan}
    #TODO: need to pass _error to build_constraint
    con_func, con_func0 = _disaggregate_nl_expression(model, con.func, bvar, method)
    ϵ = method.value
    con_func = ((1-ϵ)*bvar+ϵ)*con_func - ϵ*(1-bvar)*con_func0 - con.set.lower*bvar
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con_func, _MOI.GreaterThan(0))
    )
end
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::Hull,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
) where {T <: Union{JuMP.AffExpr, JuMP.QuadExpr}, S <: _MOI.Interval}
    #TODO: need to pass _error to build_constraint
    con_func = _disaggregate_expression(model, con.func, bvar, method)
    
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con_func - con.set.lower*bvar, _MOI.GreaterThan(0))
    )
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con_func - con.set.upper*bvar, _MOI.LessThan(0))
    )
end
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::Hull,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
) where {T <: JuMP.NonlinearExpr, S <: _MOI.Interval}
    #TODO: need to pass _error to build_constraint
    ϵ = method.value
    # reformulate LessThan Part
    con_func, con_func0 = _disaggregate_nl_expression(model, con.func, bvar, method)
    con_func = ((1-ϵ)*bvar+ϵ) * con_func - ϵ*(1-bvar)*con_func0
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con_func - con.set.upper*bvar, _MOI.LessThan(0))
    )
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con_func - con.set.lower*bvar, _MOI.GreaterThan(0))
    )
end
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::Hull,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
    ) where {T <: Union{JuMP.AffExpr, JuMP.QuadExpr}, S <: _MOI.EqualTo}
    #TODO: need to pass _error to build_constraint
    con_func = _disaggregate_expression(model, con.func, bvar, method)
    con_func -= con.set.value * bvar

    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con_func, _MOI.EqualTo(0))
    )
end
function _reformulate_disjunctive_constraints(
    model::JuMP.Model, 
    method::Hull,
    con::JuMP.ScalarConstraint{T, S}, 
    bvar::JuMP.VariableRef
    ) where {T <: JuMP.NonlinearExpr, S <: _MOI.EqualTo}
    #TODO: need to pass _error to build_constraint
    con_func, con_func0 = _disaggregate_nl_expression(model, con.func, bvar, method)
    ϵ = method.value
    con_func = ((1-ϵ)*bvar+ϵ)*con_func - ϵ*(1-bvar)*con_func0 - con.set.value*bvar
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, con_func, _MOI.EqualTo(0))
    )
end

"""

"""
function _reformulate_disjunctive_constraints(
    model::JuMP.Model,
    ::Indicator,
    con::JuMP.ScalarConstraint{JuMP.AffExpr, S},
    bvar::JuMP.VariableRef
) where {S}
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, [bvar, con.func], _MOI.Indicator{_MOI.ACTIVATE_ON_ONE}(con.set))
    )
end

# define fallbacks for other constraint types
function _reformulate_disjunctive_constraints(
    ::JuMP.Model, 
    method::AbstractReformulationMethod, 
    con::JuMP.AbstractConstraint, 
    ::JuMP.VariableRef
)
    error("$method reformulation for constraint $con is not supported yet.")
end

"""

"""
function _reformulate_logical_constraints(model::JuMP.Model)
    for (_, lcon) in logical_constraints(model)
        _reformulate_logical_constraints(model, lcon.constraint.expression)
    end
end

function _reformulate_logical_constraints(model::JuMP.Model, lexpr::LogicalExpr)
    if lexpr.head in [:exactly, :atmost, :atleast]
        _reformulate_selector(model, lexpr.head, lexpr.args[1], lexpr.args[2:end])
    end
end

function _reformulate_selector(model::JuMP.Model, kind::Symbol, val::Number, lvars::Vector{Any})
    ind_var_dict = gdp_data(model).indicator_variables
    vars = Any[ind_var_dict[var] for var in lvars]
    op = 
        kind == :exactly ? _MOI.EqualTo(val) :
        kind == :atleast ? _MOI.GreaterThan(val) :
        kind == :atmost ? _MOI.LessThan(val) :
        error("Selector operator `$(kind)` is not valid (allowed selectors are `:exactly`, `:atmost`, `:atleast`).")
    JuMP.add_constraint(model,
        JuMP.build_constraint(error, JuMP.NonlinearExpr(:+, vars), op)
    )
end

function _reformulate_selector(model::JuMP.Model, kind::Symbol, lvar::LogicalVariableRef, lvars::Vector{Any})
    ind_var_dict = gdp_data(model).indicator_variables
    var0 = ind_var_dict[lvar]
    vars = Any[ind_var_dict[v] for v in lvars]
    op = 
        kind == :exactly ? _MOI.EqualTo(0) :
        kind == :atleast ? _MOI.GreaterThan(0) :
        kind == :atmost ? _MOI.LessThan(0) :
        error("Selector operator `$(kind)` is not valid (allowed selectors are `:exactly`, `:atmost`, `:atleast`).")
    JuMP.add_constraint(model,
        build_constraint(error, JuMP.NonlinearExpr(:-, Any[JuMP.NonlinearExpr(:+, vars), var0]), op)
    )
end