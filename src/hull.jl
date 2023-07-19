"""

"""
function _get_disjunction_variables(disj::DisjunctiveConstraintData)
    vars = Set{JuMP.VariableRef}()
    for d in disj.constraint.disjuncts
        _interrogate_variables(v -> push!(vars, v), d)
    end
    return vars
end
# function _get_variables(disj::DisjunctiveConstraintData)
#     vars = Set{JuMP.VariableRef}()
#     for d in disj.constraint.disjuncts
#         union!(vars, _get_variables(d))
#     end
#     return vars
# end
# function _get_variables(d::Disjunct)
#     vars = Set{JuMP.VariableRef}()
#     for con in d.constraints
#         union!(vars, _get_variables(con))
#     end
#     return vars
# end
# function _get_variables(con::JuMP.AbstractArray{T}) where {T <: JuMP.AbstractConstraint}
#     vars = Set{JuMP.VariableRef}()
#     for c in con
#         union!(vars, _get_variables(c))
#     end
#     return vars
# end
# function _get_variables(con::JuMP.ScalarConstraint)
#     _get_variables(con.func)
# end
# function _get_variables(expr::JuMP.AffExpr)
#     return keys(expr.terms)
# end
# function _get_variables(expr::JuMP.QuadExpr)
#     vars = Set(keys(expr.aff.terms))
#     for (pair, _) in expr.terms
#         push!(vars, pair.a, pair.b)
#     end
#     return vars
# end
# function _get_variables(expr::JuMP.NonlinearExpr)
#     vars = Set{JuMP.VariableRef}()
#     _interrogate_variables(v -> push!(vars, v), expr)
#     return vars
# end
# function _get_variables(expr)
#     error("`_all_function_variables` not defined for expression of type $(typeof(expr)).")
# end

################################################################################
#                              VARIABLE ITERATION
################################################################################
## Create helper methods to interrogate the variables of an expr w/ a function
# Constant
function _interrogate_variables(interrogator::Function, c::Number)
    return
end

# VariableRef
function _interrogate_variables(interrogator::Function, var::JuMP.VariableRef)
    interrogator(var)
    return
end

# AffExpr
function _interrogate_variables(interrogator::Function, aff::JuMP.AffExpr)
    for (var, _) in aff.terms
        interrogator(var)
    end
    return
end

# QuadExpr
function _interrogate_variables(interrogator::Function, quad::JuMP.QuadExpr)
    for (pair, _) in quad.terms
        interrogator(pair.a)
        interrogator(pair.b)
    end
    _interrogate_variables(interrogator, quad.aff)
    return
end

# NonlinearExpr
function _interrogate_variables(interrogator::Function, nlp::JuMP.NonlinearExpr)
    for arg in nlp.args
        _interrogate_variables(interrogator, arg)
    end
    # See InfiniteOpt.jl for alternate method that avoids stackoverflow errors with deeply nested expressions:
    # https://github.com/infiniteopt/InfiniteOpt.jl/blob/cb6dd6ae40fe0144b1dd75da0739ea6e305d5357/src/expressions.jl#L520-L534
    return
end

# Constraint
function _interrogate_variables(interrogator::Function, con::JuMP.ScalarConstraint)
    _interrogate_variables(interrogator, con.func)
end

# AbstractArray
function _interrogate_variables(interrogator::Function, arr::AbstractArray)
    for ex in arr
        _interrogate_variables(interrogator, ex)
    end
    return
end

# Disjunct
function _interrogate_variables(interrogator::Function, d::Disjunct)
    for con in d.constraints
        _interrogate_variables(interrogator, con)
    end
    return
end

# Fallback
function _interrogate_variables(interrogator::Function, other)
    error("Cannot extract variables from object of type $(typeof(other)) inside of a disjunctive constraint.")
end


"""

"""
function _update_variable_bounds!(var_bounds_dict::Dict{JuMP.VariableRef, Tuple{Float64, Float64}}, var_set::Set{JuMP.VariableRef})
    for var in var_set
        if (var in keys(var_bounds_dict)) | JuMP.is_binary(var) #skip if binary or if bounds already stored
            continue
        elseif !JuMP.has_lower_bound(var) | !JuMP.has_upper_bound(var)
            error("Variable $var must have a lower and upper bound defined when using the Hull reformulation.")
        else
            lb = min(0, JuMP.lower_bound(var))
            ub = max(0, JuMP.upper_bound(var))
            var_bounds_dict[var] = (lb, ub)
        end
    end
end

"""

"""
function _disaggregate_variable(model::JuMP.Model, d::Disjunct, var::JuMP.VariableRef, bvar::JuMP.VariableRef)
    #create disaggregated var
    lb, ub = gdp_data(model).variable_bounds[var]
    disag_var_name = string(var, "_", d.indicator)
    disag_var = JuMP.@variable(model, 
        base_name = disag_var_name,
        lower_bound = lb,
        upper_bound = ub,
    )
    #store ref to disaggregated variable
    gdp_data(model).disaggregated_variables[Symbol(disag_var_name)] = disag_var
    #create bounding constraints
    JuMP.@constraint(model, lb*bvar - disag_var ≤ 0)
    JuMP.@constraint(model, disag_var - ub*bvar ≤ 0)

    return disag_var
end

"""

"""
function _disaggregate_expression(model::JuMP.Model, aff::JuMP.AffExpr, bvar::JuMP.VariableRef, ::Hull)
    disag_var_dict = gdp_data(model).disaggregated_variables
    new_expr = JuMP.AffExpr()
    for (var, coeff) in aff.terms
        JuMP.is_binary(var) && continue #skip binary variables
        disag_var = disag_var_dict[Symbol(var,"_",bvar)]
        new_expr.terms[disag_var] = coeff
    end

    return new_expr
end

function _disaggregate_expression(model::JuMP.Model, quad::JuMP.QuadExpr, bvar::JuMP.VariableRef, method::Hull)
    disag_var_dict = gdp_data(model).disaggregated_variables
    #get affine part
    new_expr = _disaggregate_expression(model, quad.aff, bvar, method)
    #get nonlinear part
    ϵ = method.value
    for (pair, coeff) in quad.terms
        disag_var_a = disag_var_dict[Symbol(pair.a,"_",bvar)]
        disag_var_b = disag_var_dict[Symbol(pair.b,"_",bvar)]
        new_expr += coeff * disag_var_a * disag_var_b / ((1-ϵ)*bvar+ϵ)
    end

    return new_expr
end


function _disaggregate_nl_expression(model::JuMP.Model, c::Number, ::JuMP.VariableRef, ::Hull)
    return c, c
end

function _disaggregate_nl_expression(model::JuMP.Model, var::JuMP.VariableRef, bvar::JuMP.VariableRef, method::Hull)
    ϵ = method.value
    disag_var_dict = gdp_data(model).disaggregated_variables
    new_var = disag_var_dict[Symbol(var,"_",bvar)] / ((1-ϵ)*bvar+ϵ)

    return new_var, 0
end

function _disaggregate_nl_expression(model::JuMP.Model, aff::JuMP.AffExpr, bvar::JuMP.VariableRef, ::Hull)
    disag_var_dict = gdp_data(model).disaggregated_variables
    new_expr = JuMP.AffExpr()
    for (var, coeff) in aff.terms
        JuMP.is_binary(var) && continue #skip binary variables
        disag_var = disag_var_dict[Symbol(var,"_",bvar)]
        new_expr += coef * disag_var / ((1-ϵ)*bvar+ϵ)
    end

    return new_expr, 0
end

function _disaggregate_nl_expression(model::JuMP.Model, quad::JuMP.QuadExpr, bvar::JuMP.VariableRef, method::Hull)
    disag_var_dict = gdp_data(model).disaggregated_variables
    #get affine part
    new_expr, _ = _disaggregate_nl_expression(model, quad.aff, bvar, method)
    #get nonlinear part
    ϵ = method.value
    for (pair, coeff) in quad.terms
        disag_var_a = disag_var_dict[Symbol(pair.a,"_",bvar)]
        disag_var_b = disag_var_dict[Symbol(pair.b,"_",bvar)]
        new_expr += coeff * disag_var_a * disag_var_b / ((1-ϵ)*bvar+ϵ)^2
    end

    return new_expr, 0
end

function _disaggregate_nl_expression(model::JuMP.Model, nlp::JuMP.NonlinearExpr, bvar::JuMP.VariableRef, method::Hull)
    new_args = Vector{Any}()
    new_args0 = Vector{Any}()
    for arg in nlp.args
        new_arg, new_arg0 = _disaggregate_nl_expression(model, arg, bvar, method)
        push!(new_args, new_arg)
        push!(new_args0, new_arg0)
    end
    new_expr = JuMP.NonlinearExpr(nlp.head, new_args)
    new_expr0 = eval(:($(nlp.head)($new_args0...)))
    if isinf(new_expr0)
        error("Operator `$(nlp.head)` is not defined at 0, causing the perspective function on the Hull reformulation to fail.")
    end

    return new_expr, new_expr0
end