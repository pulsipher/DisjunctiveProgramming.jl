# Example with proposition reformulation
# Proposition:
# ¬((Y[1] ∧ ¬Y[2]) ⇔ (Y[3] ∨ Y[4]))

m = GDPModel()
@variable(m, Y[1:4], LogicalVariable)
logic_1 = LogicalConstraint(
    LogicalExpr(:¬, Any[
        LogicalExpr(:⇔, Any[
                LogicalExpr(:∧, Any[
                    Y[1], 
                    LogicalExpr(:¬, Any[Y[2]])
                ]),
                LogicalExpr(:∨, Any[Y[3], Y[4]])
            ]
        )
    ])
)
selector = add_constraint(m,
    logic_1,
    "Logic Proposition"
)
DisjunctiveProgramming._reformulate_logical_variables(m)
DisjunctiveProgramming._reformulate_logical_constraints(m)
print(m)

# Feasibility
# Subject to
#  Y[3] ≥ 0
#  Y[2] ≥ 0
#  Y[1] + Y[3] + Y[4] ≥ 1
#  Y[4] ≥ 0
#  -Y[1] ≥ -1
#  -Y[1] + Y[2] - Y[3] ≥ -1
#  -Y[2] + Y[3] + Y[4] ≥ 0
#  -Y[1] + Y[2] - Y[4] ≥ -1
#  Y[1] binary
#  Y[2] binary
#  Y[3] binary
#  Y[4] binary