"""

"""
function _copy(disjunctions::_MOIUC.CleverDict{DisjunctionIndex, DisjunctiveConstraintData}, new_model::JuMP.Model)
    new_disjunctions = _MOIUC.CleverDict{DisjunctionIndex, DisjunctiveConstraintData}()
    for (_, DCD) in disjunctions
        new_disjunction = similar(DCD.constraint.disjuncts)
        for (ix, disj) in enumerate(DCD.constraint.disjuncts)
            new_disj_con = JuMP.AbstractConstraint[]
            for con in disj.constraints
                new_con = JuMP.build_constraint(error,
                    _copy(con.func, new_model),
                    con.set 
                )
                push!(new_disj_con, new_con)
            end
            new_disj = Disjunct(tuple(new_disj_con...), _copy(disj.indicator, new_model))
            new_disjunction[ix] = new_disj
        end
        new_DCD = DisjunctiveConstraintData(
            JuMP.build_constraint(error, new_disjunction), 
            DCD.name
        )
        _MOIUC.add_item(new_disjunctions, new_DCD)
    end
    
    return new_disjunctions
end

"""

"""
function _copy(logical_constraints::_MOIUC.CleverDict{LogicalConstraintIndex, LogicalConstraintData}, new_model::JuMP.Model)
    new_logical_cons = _MOIUC.CleverDict{LogicalConstraintIndex, LogicalConstraintData}()
    for (_, LCD) in logical_constraints
        new_lcon = LogicalConstraint(
            _copy(LCD.constraint.expression, new_model)
        )
        new_LCD = LogicalConstraintData(new_lcon, LCD.name)
        _MOIUC.add_item(new_logical_cons, new_LCD)
    end

    return new_logical_cons
end

"""

"""
function _copy(disaggregated_variables::Dict{Symbol, JuMP.VariableRef}, new_model::JuMP.Model)
    return Dict(
        k => _copy(v, new_model)
        for (k, v) in disaggregated_variables
    )
end

function _copy(indicator_variables::Dict{LogicalVariableRef, JuMP.VariableRef}, new_model::JuMP.Model)
    return Dict(
        _copy(k, new_model) => _copy(v, new_model)
        for (k, v) in indicator_variables
    )
end

function _copy(variable_bounds::Dict{JuMP.VariableRef, Tuple{Float64, Float64}}, new_model::JuMP.Model)
    return Dict(
        _copy(k, new_model) => v
        for (k, v) in variable_bounds
    )
end

"""

"""
# Constant
function _copy(c::Number, new_model::JuMP.Model)
    return c
end

function _copy(var::JuMP.VariableRef, new_model::JuMP.Model)
    return JuMP.VariableRef(new_model, JuMP.index(var))
end

function _copy(lvar::LogicalVariableRef, new_model::JuMP.Model)
    return LogicalVariableRef(new_model, JuMP.index(lvar))
end

function _copy(aff::JuMP.AffExpr, new_model::JuMP.Model)
    new_expr = JuMP.AffExpr()
    for (var, coeff) in aff.terms
        new_var = _copy(var, new_model)
        new_expr.terms[new_var] = coeff
    end

    return new_expr
end

"""

"""
function _copy(quad::JuMP.QuadExpr, new_model::JuMP.Model)
    new_expr = JuMP.QuadExpr()
    for (var, coeff) in quad.aff.terms
        new_var = _copy(var, new_model)
        new_expr.aff.terms[new_var] = coeff
    end
    for (pair, coeff) in quad.terms
        vara = JuMP.VariableRef(new_model, JuMP.index(pair.a))
        varb = JuMP.VariableRef(new_model, JuMP.index(pair.b))
        new_term = JuMP.UnorderedPair{JuMP.VariableRef}(vara, varb)
        new_expr.terms[new_term] = coeff
    end

    return new_expr
end

"""

"""
function _copy(nlp::JuMP.NonlinearExpr, new_model::JuMP.Model)
    #TODO: use stack to avoid recursion stackoverflow error for deeply nested expression
    new_args = Vector{Any}()
    for arg in nlp.args
        push!(new_args, _copy(arg, new_model))
    end

    return JuMP.NonlinearExpr(nlp.head, new_args)
end