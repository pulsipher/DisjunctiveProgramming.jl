################################################################################
#                              LOGIC OPERATORS
################################################################################
function _op_fallback(name)
    error("`$name` is only supported for logical expressions")
end

# Define all the logical operators
for (name, alt) in (
    (:⇔, :iff), # \Leftrightarrow + tab
    (:⟹, :implies) # Longrightarrow + tab
    )
    # make operators
    @eval begin 
        const $name = JuMP.NonlinearOperator((vs...) -> _op_fallback($(Meta.quot(name))), $(Meta.quot(name)))
        const $alt = JuMP.NonlinearOperator((vs...) -> _op_fallback($(Meta.quot(alt))), $(Meta.quot(name)))
    end
end
for (name, alt, func) in (
    (:∨, :logical_or, :(|)), # \vee + tab
    (:∧, :logical_and, :(&)), # \wedge + tab
    (:¬, :logical_negation, :(!)) # \neg + tab
    )
    # make operators
    @eval begin 
        const $name = JuMP.NonlinearOperator($func, $(Meta.quot(name)))
        const $alt = JuMP.NonlinearOperator($func, $(Meta.quot(name)))
    end
end

################################################################################
#                            CONJUNCTIVE NORMAL FORM
################################################################################
function _to_cnf(lexpr::_LogicalExpr)
    #NOTE: some redundant constraints may be created in the process.
    #   For example A ∨ ¬B ∨ B is always true and is reformulated 
    #   to the redundant constraint A ≥ 0.
    lexpr |> 
        _eliminate_equivalence |>  
        _eliminate_implication |> 
        _move_negations_inward |> 
        _distribute_and_over_or |>
        _flatten
end

# Eliminate the equivalence operator `⇔` by replacing it with two implications.
function _eliminate_equivalence(lvar::LogicalVariableRef)
    return lvar
end
function _eliminate_equivalence(lexpr::_LogicalExpr)
    if lexpr.head == :⇔
        A = _eliminate_equivalence(lexpr.args[1])
        if length(lexpr.args) > 2 
            nested = _LogicalExpr(:⇔, Vector{Any}(lexpr.args[2:end]))
            B = _eliminate_equivalence(nested)
        elseif length(lexpr.args) == 2
            B = _eliminate_equivalence(lexpr.args[2])
        else
            error("The equivalence logic operator `⇔` must have at least two arguments.")
        end
        new_lexpr = _LogicalExpr(:∧, Any[
            _LogicalExpr(:⟹, Any[A, B]),
            _LogicalExpr(:⟹, Any[B, A])
        ])
    else
        new_lexpr = _LogicalExpr(lexpr.head, Any[
            _eliminate_equivalence(arg) for arg in lexpr.args
        ])
    end

    return new_lexpr
end

# Eliminate the implication operator `⟹` by replacing it with a disjunction.
function _eliminate_implication(lvar::LogicalVariableRef)
    return lvar
end
function _eliminate_implication(lexpr::_LogicalExpr)
    if lexpr.head == :⟹
        if length(lexpr.args) != 2 
            error("The implication operator must have two clauses.")
        end
        A = _eliminate_implication(lexpr.args[1])
        B = _eliminate_implication(lexpr.args[2])
        new_lexpr = _LogicalExpr(:∨, Any[
            _LogicalExpr(:¬, Any[A]),
            B
        ])
    else
        new_lexpr = _LogicalExpr(lexpr.head, Any[
            _eliminate_implication(arg) for arg in lexpr.args
        ])
    end

    return new_lexpr
end

# Move negations inward by applying De Morgan's laws.
function _move_negations_inward(lvar::LogicalVariableRef)
    return lvar
end
function _move_negations_inward(lexpr::_LogicalExpr)
    if lexpr.head == :¬
        if length(lexpr.args) != 1
            error("The negation operator can only have 1 clause.")
        end
        new_lexpr = _negate(lexpr.args[1])
    else
        new_lexpr = _LogicalExpr(lexpr.head, Any[
            _move_negations_inward(arg) for arg in lexpr.args
        ])
    end

    return new_lexpr
end

function _negate(lvar::LogicalVariableRef)
    return _LogicalExpr(:¬, Any[lvar])
end
function _negate(lexpr::_LogicalExpr)
    if lexpr.head == :∨
        return _negate_or(lexpr)
    elseif lexpr.head == :∧
        return _negate_and(lexpr)
    elseif lexpr.head == :¬
        return _negate_negation(lexpr)
    else
        error("Unexpected operator `$(lexpr.head)`in logic expression.")
    end
end

function _negate_or(lexpr::_LogicalExpr)
    if length(lexpr.args) < 2 
        error("The OR operator must have at least two clauses.")
    end
    return _LogicalExpr(:∧, Any[ #flip OR to AND
        _move_negations_inward(_LogicalExpr(:¬, Any[arg]))
        for arg in lexpr.args
    ])
end

function _negate_and(lexpr::_LogicalExpr)
    if length(lexpr.args) < 2 
        error("The AND operator must have at least two clauses.")
    end
    return _LogicalExpr(:∨, Any[ #flip AND to OR
        _move_negations_inward(_LogicalExpr(:¬, Any[arg]))
        for arg in lexpr.args
    ])
end

function _negate_negation(lexpr::_LogicalExpr)
    if length(lexpr.args) != 1
        error("The negation operator can only have 1 clause.")
    end
    return _move_negations_inward(lexpr.args[1])
end

function _distribute_and_over_or(lvar::LogicalVariableRef)
    return lvar
end
function _distribute_and_over_or(lexpr0::_LogicalExpr)
    lexpr = _flatten(lexpr0)
    if lexpr.head == :∨
        if length(lexpr.args) < 2 
            error("The OR operator must have at least two clauses.")
        end
        loc = findfirst(arg -> arg isa _LogicalExpr ? arg.head == :∧ : false, lexpr.args)
        if !isnothing(loc)
            new_lexpr = _LogicalExpr(:∧, Any[
                _distribute_and_over_or(
                    _LogicalExpr(:∨, Any[arg_i, lexpr.args[setdiff(1:end,loc)]...])
                )
                for arg_i in lexpr.args[loc].args
            ])
        else
            new_lexpr = lexpr
        end
    else
        new_lexpr = _LogicalExpr(lexpr.head, Any[
            _distribute_and_over_or(arg) for arg in lexpr.args
        ])
    end

    return new_lexpr
end

# Flatten netsed OR / AND operators and replace them with their n-ary form.
#   For example, ∨(∨(A, B), C) is replaced with ∨(A, B, C).
function _flatten(lvar::LogicalVariableRef)
    return lvar
end
function _flatten(lexpr::_LogicalExpr)
    if lexpr.head in (:∧, :∨)
        nary_args = Set{Any}()
        for arg in lexpr.args
            if arg isa LogicalVariableRef
                push!(nary_args, arg)
            elseif _isa_literal(arg)
                push!(nary_args, arg)
            elseif arg.head == lexpr.head
                arg_flat = _flatten(arg)
                for a in arg_flat.args
                    push!(nary_args, _flatten(a))
                end
            else
                arg_flat = _flatten(arg)
                push!(nary_args, arg_flat)
            end
        end
        new_lexpr = _LogicalExpr(lexpr.head, collect(nary_args))
    else 
        new_lexpr = _LogicalExpr(lexpr.head, Any[
            _flatten(arg) for arg in lexpr.args
        ])
    end
    return new_lexpr
end

################################################################################
#                              SELECTOR REFORMULATION
################################################################################
# cardinality constraint reformulation
function _reformulate_selector(model::JuMP.Model, ::_MOIAtLeast, val::Number, lvrefs::Vector{LogicalVariableRef})
    bvrefs = _indicator_to_binary_ref.(lvrefs)
    reform_con = JuMP.add_constraint(model,
        JuMP.build_constraint(error, JuMP.@expression(model, sum(bvrefs)), _MOI.GreaterThan(val))
    )
    push!(_reformulation_constraints(model), (JuMP.index(reform_con), JuMP.ScalarShape()))
end
function _reformulate_selector(model::JuMP.Model, ::_MOIAtMost, val::Number, lvrefs::Vector{LogicalVariableRef})
    bvrefs = _indicator_to_binary_ref.(lvrefs)
    reform_con = JuMP.add_constraint(model,
        JuMP.build_constraint(error, JuMP.@expression(model, sum(bvrefs)), _MOI.LessThan(val))
    )
    push!(_reformulation_constraints(model), (JuMP.index(reform_con), JuMP.ScalarShape()))
end
function _reformulate_selector(model::JuMP.Model, ::_MOIExactly, val::Number, lvrefs::Vector{LogicalVariableRef})
    bvrefs = _indicator_to_binary_ref.(lvrefs)
    reform_con = JuMP.add_constraint(model,
        JuMP.build_constraint(error, JuMP.@expression(model, sum(bvrefs)), _MOI.EqualTo(val))
    )
    push!(_reformulation_constraints(model), (JuMP.index(reform_con), JuMP.ScalarShape()))
end
function _reformulate_selector(model::JuMP.Model, ::_MOIAtLeast, lvref::LogicalVariableRef, lvrefs::Vector{LogicalVariableRef})
    bvref = _indicator_to_binary_ref(lvref)
    bvrefs = Vector{Any}(_indicator_to_binary_ref.(lvrefs))
    reform_con = JuMP.add_constraint(model,
        JuMP.build_constraint(error, JuMP.@expression(model, sum(bvrefs) - bvref), _MOI.GreaterThan(0))
    )
    push!(_reformulation_constraints(model), (JuMP.index(reform_con), JuMP.ScalarShape()))
end
function _reformulate_selector(model::JuMP.Model, ::_MOIAtMost, lvref::LogicalVariableRef, lvrefs::Vector{LogicalVariableRef})
    bvref = _indicator_to_binary_ref(lvref)
    bvrefs = Vector{Any}(_indicator_to_binary_ref.(lvrefs))
    reform_con = JuMP.add_constraint(model,
        JuMP.build_constraint(error, JuMP.@expression(model, sum(bvrefs) - bvref), _MOI.LessThan(0))
    )
    push!(_reformulation_constraints(model), (JuMP.index(reform_con), JuMP.ScalarShape()))
end
function _reformulate_selector(model::JuMP.Model, ::_MOIExactly, lvref::LogicalVariableRef, lvrefs::Vector{LogicalVariableRef})
    bvref = _indicator_to_binary_ref(lvref)
    bvrefs = Vector{Any}(_indicator_to_binary_ref.(lvrefs))
    reform_con = JuMP.add_constraint(model,
        JuMP.build_constraint(error, JuMP.@expression(model, sum(bvrefs) - bvref), _MOI.EqualTo(0))
    )
    push!(_reformulation_constraints(model), (JuMP.index(reform_con), JuMP.ScalarShape()))
end

################################################################################
#                              PROPOSITION REFORMULATION
################################################################################
function _reformulate_proposition(model::JuMP.Model, lexpr::_LogicalExpr)
    expr = _to_cnf(lexpr)
    if expr.head == :∧
        for arg in expr.args
            _add_reformulated_proposition(model, arg)
        end
    elseif expr.head == :∨ && all(_isa_literal.(expr.args))
        _add_reformulated_proposition(model, expr)
    else
        error("Expression was not converted to proper Conjunctive Normal Form:\n$expr")
    end
end

# helper to determine if an object is a logic literal (i.e. a logic variable or its negation)
_isa_literal(v::LogicalVariableRef) = true
_isa_literal(v::_LogicalExpr) = (v.head == :¬) && (length(v.args) == 1) && _isa_literal(v.args[1])
_isa_literal(v) = false

function _add_reformulated_proposition(model::JuMP.Model, arg::Union{LogicalVariableRef,_LogicalExpr})
    func = _reformulate_clause(model, arg)
    if !isempty(func.terms) && !all(iszero.(values(func.terms)))
        con = JuMP.build_constraint(error, func, _MOI.GreaterThan(1))
        reform_con = JuMP.add_constraint(model, con)
        push!(_reformulation_constraints(model), (JuMP.index(reform_con), JuMP.ScalarShape()))
    end
    return
end

function _reformulate_clause(model::JuMP.Model, lvref::LogicalVariableRef)
    func = 1 * _indicator_to_binary_ref(lvref)
    return func
end

function _reformulate_clause(model::JuMP.Model, lexpr::_LogicalExpr)
    func = zero(JuMP.AffExpr) #initialize func expression
    if _isa_literal(lexpr)
        JuMP.add_to_expression!(func, 1 - _reformulate_clause(model, lexpr.args[1]))
    elseif lexpr.head == :∨
        for literal in lexpr.args
            if literal isa LogicalVariableRef
                JuMP.add_to_expression!(func, _reformulate_clause(model, literal))
            elseif _isa_literal(literal)
                JuMP.add_to_expression!(func, 1 - _reformulate_clause(model, literal.args[1]))
            else
                error("Expression was not converted to proper Conjunctive Normal Form:\n$literal is not a literal.")
            end
        end
    else
        error("Expression was not converted to proper Conjunctive Normal Form:\n$lexpr.")
    end
    
    return func
end