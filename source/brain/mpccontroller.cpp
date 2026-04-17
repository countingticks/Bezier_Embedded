#include <brain/mpccontroller.hpp>

#include <cmath>
#include <cstring>

namespace
{
    static const float PI_FLOAT = 3.14159265358979323846f;
    static const float STEER_TO_YAW_SIGN = -1.0f;
    static const float HESSIAN_REGULARIZATION = 1e-5f;
    static const float CONSTRAINT_TOLERANCE = 1e-4f;
    static const float ITERATION_ACCEPT_VIOLATION = 5e-2f;
    static const float UPDATE_TOLERANCE = 5e-4f;
    static const float MIN_STEP_SIZE = 1e-3f;
    static const uint8_t PROJECTION_SWEEPS = 1U;
    static const float DISTURBANCE_HEADING_RESET_RAD = 0.785398163f;
    static const float DISTURBANCE_LATERAL_RESET_M = 0.10f;
    static const float DISTURBANCE_PROGRESS_RESET_M = 0.20f;

    static const float STATE_WEIGHTS[brain::CMpcController::c_stateCount] = {
        brain::CMpcController::c_qES / (brain::CMpcController::c_eSScaleM * brain::CMpcController::c_eSScaleM),
        brain::CMpcController::c_qEY / (brain::CMpcController::c_eYScaleM * brain::CMpcController::c_eYScaleM),
        brain::CMpcController::c_qEPsi / (brain::CMpcController::c_ePsiScaleRad * brain::CMpcController::c_ePsiScaleRad)
    };

    static const float TERMINAL_WEIGHTS[brain::CMpcController::c_stateCount] = {
        brain::CMpcController::c_pES / (brain::CMpcController::c_eSScaleM * brain::CMpcController::c_eSScaleM),
        brain::CMpcController::c_pEY / (brain::CMpcController::c_eYScaleM * brain::CMpcController::c_eYScaleM),
        brain::CMpcController::c_pEPsi / (brain::CMpcController::c_ePsiScaleRad * brain::CMpcController::c_ePsiScaleRad)
    };

    static const float INPUT_WEIGHTS[brain::CMpcController::c_inputCount] = {
        brain::CMpcController::c_rV,
        brain::CMpcController::c_rDelta
    };

    static const float SMOOTHNESS_WEIGHTS[brain::CMpcController::c_inputCount] = {
        brain::CMpcController::c_sV,
        brain::CMpcController::c_sDelta
    };

    static const float INPUT_SCALES[brain::CMpcController::c_inputCount] = {
        brain::CMpcController::c_vCorrScaleMps,
        brain::CMpcController::c_deltaCorrScaleRad
    };

    static bool isFiniteFloat(float f_value)
    {
        return std::isfinite(f_value) != 0;
    }

    static uint16_t saturatingAddUint16(uint16_t f_left, uint16_t f_right)
    {
        const uint32_t accumulator = static_cast<uint32_t>(f_left) + static_cast<uint32_t>(f_right);
        return (accumulator > 0xFFFFU) ? 0xFFFFU : static_cast<uint16_t>(accumulator);
    }

    static bool canAcceptIterationLimitedSolution(
        brain::CMpcController::SolverStatus f_status,
        float f_objective,
        float f_maxViolation
    )
    {
        return
            (f_status == brain::CMpcController::SolverStatus::IterationLimit) &&
            isFiniteFloat(f_objective) &&
            isFiniteFloat(f_maxViolation) &&
            (f_maxViolation <= ITERATION_ACCEPT_VIOLATION);
    }
}

namespace brain
{
    CMpcController::Output::Output()
        : v_corr_mps(0.0f)
        , delta_corr_rad(0.0f)
        , path_speed_cmd_mps(0.0f)
        , body_speed_cmd_mps(0.0f)
        , steer_cmd_rad(0.0f)
        , e_s_m(0.0f)
        , e_y_m(0.0f)
        , e_psi_rad(0.0f)
        , objective(0.0f)
        , max_constraint_violation(0.0f)
        , iterations(0U)
        , status(SolverStatus::Disabled)
        , used_previous_correction(false)
        , used_feedforward_only(true)
        , speed_rate_limited(false)
        , steer_rate_limited(false)
        , success(false)
    {
    }

    CMpcController::CMpcController()
        : m_hasWarmStart(false)
        , m_warmStart()
        , m_hasPreviousCorrection(false)
        , m_previousCorrectionNormalized()
        , m_hessian()
        , m_gradient()
        , m_constraintMatrix()
        , m_constraintBounds()
    {
    }

    void CMpcController::reset()
    {
        resetWarmStart();
    }

    void CMpcController::resetWarmStart()
    {
        m_hasWarmStart = false;
        memset(m_warmStart, 0, sizeof(m_warmStart));
        clearPreviousCorrection();
    }

    CMpcController::Output CMpcController::solve(const Input& f_input)
    {
        float currentState[c_stateCount] = {0.0f, 0.0f, 0.0f};
        buildCurrentState(f_input, currentState);

        if (!f_input.feedback_enabled || !f_input.dynamic_segment)
        {
            return makeOutput(
                f_input,
                SolverStatus::Disabled,
                false,
                true,
                currentState,
                0.0f,
                0.0f,
                0U,
                0.0f,
                0.0f
            );
        }

        if (!f_input.pose.valid || (f_input.direction == 0))
        {
            return makeOutput(
                f_input,
                SolverStatus::FeedforwardOnly,
                false,
                true,
                currentState,
                0.0f,
                0.0f,
                0U,
                0.0f,
                0.0f
            );
        }

        if (shouldInvalidateForLargeDisturbance(currentState))
        {
            resetWarmStart();
        }

        float previousCorrectionNormalized[c_inputCount] = {0.0f, 0.0f};
        if (m_hasPreviousCorrection)
        {
            memcpy(previousCorrectionNormalized, m_previousCorrectionNormalized, sizeof(previousCorrectionNormalized));
        }

        size_t constraintCount = 0U;
        float stepSize = MIN_STEP_SIZE;
        float solution[c_decisionCount];
        uint16_t primaryIterations = 0U;
        float primaryMaxViolation = 0.0f;
        SolverStatus primaryStatus = SolverStatus::NumericalFailure;

        if (buildCondensedProblem(f_input, currentState, previousCorrectionNormalized, constraintCount, stepSize))
        {
            shiftWarmStart(solution);
            projectConstraints(solution, constraintCount);
            primaryStatus = solveProjectedGradient(solution, constraintCount, stepSize, primaryIterations, primaryMaxViolation);

            const float objective = computeObjective(solution);
            if ((primaryStatus == SolverStatus::Success) || canAcceptIterationLimitedSolution(primaryStatus, objective, primaryMaxViolation))
            {
                memcpy(m_warmStart, solution, sizeof(m_warmStart));
                m_hasWarmStart = true;

                const float vCorrMps = solution[0] * INPUT_SCALES[0];
                const float deltaCorrRad = solution[1] * INPUT_SCALES[1];
                updatePreviousCorrection(vCorrMps, deltaCorrRad);

                return makeOutput(
                    f_input,
                    primaryStatus,
                    false,
                    false,
                    currentState,
                    vCorrMps,
                    deltaCorrRad,
                    primaryIterations,
                    objective,
                    primaryMaxViolation
                );
            }
        }

        resetWarmStart();

        const float zeroCorrectionNormalized[c_inputCount] = {0.0f, 0.0f};
        float retrySolution[c_decisionCount] = {0.0f};
        size_t retryConstraintCount = 0U;
        float retryStepSize = MIN_STEP_SIZE;
        uint16_t retryIterations = 0U;
        float retryMaxViolation = 0.0f;
        SolverStatus retryStatus = SolverStatus::NumericalFailure;

        if (buildCondensedProblem(f_input, currentState, zeroCorrectionNormalized, retryConstraintCount, retryStepSize))
        {
            projectConstraints(retrySolution, retryConstraintCount);
            retryStatus = solveProjectedGradient(retrySolution, retryConstraintCount, retryStepSize, retryIterations, retryMaxViolation);

            const float objective = computeObjective(retrySolution);
            if ((retryStatus == SolverStatus::Success) || canAcceptIterationLimitedSolution(retryStatus, objective, retryMaxViolation))
            {
                memcpy(m_warmStart, retrySolution, sizeof(m_warmStart));
                m_hasWarmStart = true;

                const float vCorrMps = retrySolution[0] * INPUT_SCALES[0];
                const float deltaCorrRad = retrySolution[1] * INPUT_SCALES[1];
                updatePreviousCorrection(vCorrMps, deltaCorrRad);

                return makeOutput(
                    f_input,
                    retryStatus,
                    false,
                    false,
                    currentState,
                    vCorrMps,
                    deltaCorrRad,
                    saturatingAddUint16(primaryIterations, retryIterations),
                    objective,
                    retryMaxViolation
                );
            }
        }

        const float currentVRefPathMps = fabsf(f_input.current.v_body_mps);
        const float maxDeltaSpeed = c_sampleTimeS * f_input.limits.speed_rate_max_mps2;
        const float maxDeltaSteer = c_sampleTimeS * f_input.limits.steer_rate_max_rad_s;
        const float releasedPathSpeedMps = clampFloat(
            moveToward(
                f_input.prev_path_speed_cmd_mps,
                clampFloat(currentVRefPathMps, f_input.limits.path_speed_min_mps, f_input.limits.path_speed_max_mps),
                maxDeltaSpeed
            ),
            f_input.limits.path_speed_min_mps,
            f_input.limits.path_speed_max_mps
        );
        const float releasedSteerRad = clampFloat(
            moveToward(
                f_input.prev_steer_cmd_rad,
                f_input.current.delta_ff_rad,
                maxDeltaSteer
            ),
            f_input.limits.steer_min_rad,
            f_input.limits.steer_max_rad
        );
        const float maxViolation = (primaryMaxViolation > retryMaxViolation) ? primaryMaxViolation : retryMaxViolation;

        return makeOutput(
            f_input,
            SolverStatus::RecoveryRelease,
            false,
            false,
            currentState,
            releasedPathSpeedMps - currentVRefPathMps,
            releasedSteerRad - f_input.current.delta_ff_rad,
            saturatingAddUint16(primaryIterations, retryIterations),
            0.0f,
            maxViolation
        );
    }

    float CMpcController::wrapAngle(float f_angle)
    {
        while (f_angle > PI_FLOAT)
        {
            f_angle -= (2.0f * PI_FLOAT);
        }

        while (f_angle < -PI_FLOAT)
        {
            f_angle += (2.0f * PI_FLOAT);
        }

        return f_angle;
    }

    float CMpcController::clampFloat(float f_value, float f_minValue, float f_maxValue)
    {
        if (f_value < f_minValue)
        {
            return f_minValue;
        }

        if (f_value > f_maxValue)
        {
            return f_maxValue;
        }

        return f_value;
    }

    float CMpcController::moveToward(float f_current, float f_target, float f_maxStep)
    {
        const float limitedStep = (f_maxStep < 0.0f) ? 0.0f : f_maxStep;
        if (f_target > (f_current + limitedStep))
        {
            return f_current + limitedStep;
        }

        if (f_target < (f_current - limitedStep))
        {
            return f_current - limitedStep;
        }

        return f_target;
    }

    void CMpcController::clearProblem()
    {
        memset(m_hessian, 0, sizeof(m_hessian));
        memset(m_gradient, 0, sizeof(m_gradient));
        memset(m_constraintMatrix, 0, sizeof(m_constraintMatrix));
        memset(m_constraintBounds, 0, sizeof(m_constraintBounds));
    }

    void CMpcController::shiftWarmStart(float (&f_shiftedSolution)[c_decisionCount]) const
    {
        if (!m_hasWarmStart)
        {
            memset(f_shiftedSolution, 0, sizeof(f_shiftedSolution));
            return;
        }

        for (size_t index = 0U; index < (c_decisionCount - c_inputCount); index++)
        {
            f_shiftedSolution[index] = m_warmStart[index + c_inputCount];
        }

        f_shiftedSolution[c_decisionCount - 2U] = m_warmStart[c_decisionCount - 2U];
        f_shiftedSolution[c_decisionCount - 1U] = m_warmStart[c_decisionCount - 1U];
    }

    void CMpcController::clearPreviousCorrection()
    {
        m_hasPreviousCorrection = false;
        memset(m_previousCorrectionNormalized, 0, sizeof(m_previousCorrectionNormalized));
    }

    void CMpcController::buildCurrentState(const Input& f_input, float (&f_state)[c_stateCount]) const
    {
        f_state[0] = f_input.current.matched_progress_m - f_input.current.nominal_progress_m;

        if (!f_input.pose.valid)
        {
            f_state[1] = 0.0f;
            f_state[2] = 0.0f;
            return;
        }

        const float deltaX = f_input.pose.x_m - f_input.current.x_m;
        const float deltaY = f_input.pose.y_m - f_input.current.y_m;
        const float cosTheta = cosf(f_input.current.theta_rad);
        const float sinTheta = sinf(f_input.current.theta_rad);

        f_state[1] = (-sinTheta * deltaX) + (cosTheta * deltaY);
        f_state[2] = wrapAngle(f_input.pose.psi_rad - f_input.current.theta_rad);
    }

    bool CMpcController::shouldInvalidateForLargeDisturbance(const float (&f_state)[c_stateCount]) const
    {
        return
            (fabsf(f_state[0]) >= DISTURBANCE_PROGRESS_RESET_M) ||
            (fabsf(f_state[1]) >= DISTURBANCE_LATERAL_RESET_M) ||
            (fabsf(f_state[2]) >= DISTURBANCE_HEADING_RESET_RAD);
    }

    bool CMpcController::buildCondensedProblem(
        const Input& f_input,
        const float (&f_state)[c_stateCount],
        const float (&f_previousCorrectionNormalized)[c_inputCount],
        size_t& f_constraintCount,
        float& f_stepSize
    )
    {
        clearProblem();
        f_constraintCount = 0U;

        float stateConst[c_stateCount] = {
            f_state[0],
            f_state[1],
            f_state[2]
        };
        float stateMap[c_stateCount][c_decisionCount];
        memset(stateMap, 0, sizeof(stateMap));

        float previousVRefPathMps = 0.0f;
        float previousDeltaFfRad = 0.0f;

        for (size_t stageIndex = 0U; stageIndex < c_horizonLength; stageIndex++)
        {
            const float vRefPathMps = fabsf(f_input.horizon[stageIndex].v_body_mps);
            const float deltaFfRad = f_input.horizon[stageIndex].delta_ff_rad;
            const float cosDelta = cosf(deltaFfRad);
            const float safeCosDelta = (fabsf(cosDelta) < 0.25f) ? ((cosDelta < 0.0f) ? -0.25f : 0.25f) : cosDelta;
            const float secSquared = 1.0f / (safeCosDelta * safeCosDelta);
            const float directionSign = static_cast<float>(f_input.direction);
            const float kappaRef = STEER_TO_YAW_SIGN * directionSign * tanf(deltaFfRad) / c_wheelbaseM;

            float aMatrix[c_stateCount][c_stateCount] = {
                {
                    1.0f,
                    c_sampleTimeS * kappaRef * vRefPathMps,
                    0.0f
                },
                {
                    0.0f,
                    1.0f,
                    c_sampleTimeS * vRefPathMps
                },
                {
                    0.0f,
                    -c_sampleTimeS * kappaRef * kappaRef * vRefPathMps,
                    1.0f
                }
            };

            float bMatrix[c_stateCount][c_inputCount] = {
                {
                    c_sampleTimeS * INPUT_SCALES[0],
                    0.0f
                },
                {
                    0.0f,
                    0.0f
                },
                {
                    0.0f,
                    c_sampleTimeS * STEER_TO_YAW_SIGN * directionSign * (vRefPathMps / c_wheelbaseM) * secSquared * INPUT_SCALES[1]
                }
            };

            float nextConst[c_stateCount];
            float nextMap[c_stateCount][c_decisionCount];

            for (size_t row = 0U; row < c_stateCount; row++)
            {
                nextConst[row] = 0.0f;
                for (size_t column = 0U; column < c_stateCount; column++)
                {
                    nextConst[row] += aMatrix[row][column] * stateConst[column];
                }

                for (size_t decision = 0U; decision < c_decisionCount; decision++)
                {
                    nextMap[row][decision] = 0.0f;
                    for (size_t column = 0U; column < c_stateCount; column++)
                    {
                        nextMap[row][decision] += aMatrix[row][column] * stateMap[column][decision];
                    }
                }

                const size_t decisionBase = stageIndex * c_inputCount;
                nextMap[row][decisionBase] += bMatrix[row][0];
                nextMap[row][decisionBase + 1U] += bMatrix[row][1];
            }

            const float* stateWeights = (stageIndex + 1U < c_horizonLength) ? STATE_WEIGHTS : TERMINAL_WEIGHTS;
            for (size_t decisionRow = 0U; decisionRow < c_decisionCount; decisionRow++)
            {
                for (size_t decisionColumn = 0U; decisionColumn < c_decisionCount; decisionColumn++)
                {
                    float accumulator = 0.0f;
                    for (size_t stateIndex = 0U; stateIndex < c_stateCount; stateIndex++)
                    {
                        accumulator += nextMap[stateIndex][decisionRow] * stateWeights[stateIndex] * nextMap[stateIndex][decisionColumn];
                    }
                    m_hessian[decisionRow][decisionColumn] += (2.0f * accumulator);
                }

                float gradientAccumulator = 0.0f;
                for (size_t stateIndex = 0U; stateIndex < c_stateCount; stateIndex++)
                {
                    gradientAccumulator += nextMap[stateIndex][decisionRow] * stateWeights[stateIndex] * nextConst[stateIndex];
                }
                m_gradient[decisionRow] += (2.0f * gradientAccumulator);
            }

            const size_t decisionBase = stageIndex * c_inputCount;
            m_hessian[decisionBase][decisionBase] += (2.0f * INPUT_WEIGHTS[0]);
            m_hessian[decisionBase + 1U][decisionBase + 1U] += (2.0f * INPUT_WEIGHTS[1]);

            if (stageIndex == 0U)
            {
                m_hessian[decisionBase][decisionBase] += (2.0f * SMOOTHNESS_WEIGHTS[0]);
                m_hessian[decisionBase + 1U][decisionBase + 1U] += (2.0f * SMOOTHNESS_WEIGHTS[1]);
                m_gradient[decisionBase] -= (2.0f * SMOOTHNESS_WEIGHTS[0] * f_previousCorrectionNormalized[0]);
                m_gradient[decisionBase + 1U] -= (2.0f * SMOOTHNESS_WEIGHTS[1] * f_previousCorrectionNormalized[1]);
            }
            else
            {
                const size_t previousDecisionBase = decisionBase - c_inputCount;

                m_hessian[decisionBase][decisionBase] += (2.0f * SMOOTHNESS_WEIGHTS[0]);
                m_hessian[previousDecisionBase][previousDecisionBase] += (2.0f * SMOOTHNESS_WEIGHTS[0]);
                m_hessian[decisionBase][previousDecisionBase] -= (2.0f * SMOOTHNESS_WEIGHTS[0]);
                m_hessian[previousDecisionBase][decisionBase] -= (2.0f * SMOOTHNESS_WEIGHTS[0]);

                m_hessian[decisionBase + 1U][decisionBase + 1U] += (2.0f * SMOOTHNESS_WEIGHTS[1]);
                m_hessian[previousDecisionBase + 1U][previousDecisionBase + 1U] += (2.0f * SMOOTHNESS_WEIGHTS[1]);
                m_hessian[decisionBase + 1U][previousDecisionBase + 1U] -= (2.0f * SMOOTHNESS_WEIGHTS[1]);
                m_hessian[previousDecisionBase + 1U][decisionBase + 1U] -= (2.0f * SMOOTHNESS_WEIGHTS[1]);
            }

            appendCommandConstraints(f_input, stageIndex, vRefPathMps, deltaFfRad, f_constraintCount);
            appendRateConstraints(
                f_input,
                stageIndex,
                vRefPathMps,
                deltaFfRad,
                (stageIndex == 0U) ? f_input.prev_path_speed_cmd_mps : previousVRefPathMps,
                (stageIndex == 0U) ? f_input.prev_steer_cmd_rad : previousDeltaFfRad,
                f_constraintCount
            );

            previousVRefPathMps = vRefPathMps;
            previousDeltaFfRad = deltaFfRad;

            memcpy(stateConst, nextConst, sizeof(stateConst));
            memcpy(stateMap, nextMap, sizeof(stateMap));
        }

        float maxRowSum = 0.0f;
        for (size_t row = 0U; row < c_decisionCount; row++)
        {
            m_hessian[row][row] += HESSIAN_REGULARIZATION;

            float rowSum = 0.0f;
            for (size_t column = 0U; column < c_decisionCount; column++)
            {
                if (!isFiniteFloat(m_hessian[row][column]))
                {
                    return false;
                }
                rowSum += fabsf(m_hessian[row][column]);
            }
            if (!isFiniteFloat(m_gradient[row]))
            {
                return false;
            }
            if (rowSum > maxRowSum)
            {
                maxRowSum = rowSum;
            }
        }

        if (maxRowSum < MIN_STEP_SIZE)
        {
            maxRowSum = 1.0f;
        }

        f_stepSize = 0.8f / maxRowSum;
        if (f_stepSize < MIN_STEP_SIZE)
        {
            f_stepSize = MIN_STEP_SIZE;
        }

        return true;
    }

    void CMpcController::appendCommandConstraints(const Input& f_input, size_t f_stageIndex, float f_vRefPathMps, float f_deltaFfRad, size_t& f_constraintCount)
    {
        const size_t decisionBase = f_stageIndex * c_inputCount;

        addConstraint(
            f_constraintCount++,
            decisionBase,
            INPUT_SCALES[0],
            f_input.limits.path_speed_max_mps - f_vRefPathMps
        );
        addConstraint(
            f_constraintCount++,
            decisionBase,
            -INPUT_SCALES[0],
            f_vRefPathMps - f_input.limits.path_speed_min_mps
        );

        addConstraint(
            f_constraintCount++,
            decisionBase + 1U,
            INPUT_SCALES[1],
            f_input.limits.steer_max_rad - f_deltaFfRad
        );
        addConstraint(
            f_constraintCount++,
            decisionBase + 1U,
            -INPUT_SCALES[1],
            f_deltaFfRad - f_input.limits.steer_min_rad
        );
    }

    void CMpcController::appendRateConstraints(
        const Input& f_input,
        size_t f_stageIndex,
        float f_vRefPathMps,
        float f_deltaFfRad,
        float f_prevVRefPathMps,
        float f_prevDeltaFfRad,
        size_t& f_constraintCount
    )
    {
        const size_t decisionBase = f_stageIndex * c_inputCount;
        const float maxDeltaSpeed = c_sampleTimeS * f_input.limits.speed_rate_max_mps2;
        const float maxDeltaSteer = c_sampleTimeS * f_input.limits.steer_rate_max_rad_s;

        if (f_stageIndex == 0U)
        {
            addConstraint(
                f_constraintCount++,
                decisionBase,
                INPUT_SCALES[0],
                maxDeltaSpeed + f_prevVRefPathMps - f_vRefPathMps
            );
            addConstraint(
                f_constraintCount++,
                decisionBase,
                -INPUT_SCALES[0],
                maxDeltaSpeed - f_prevVRefPathMps + f_vRefPathMps
            );

            addConstraint(
                f_constraintCount++,
                decisionBase + 1U,
                INPUT_SCALES[1],
                maxDeltaSteer + f_prevDeltaFfRad - f_deltaFfRad
            );
            addConstraint(
                f_constraintCount++,
                decisionBase + 1U,
                -INPUT_SCALES[1],
                maxDeltaSteer - f_prevDeltaFfRad + f_deltaFfRad
            );
            return;
        }

        const size_t previousDecisionBase = decisionBase - c_inputCount;

        addConstraint(
            f_constraintCount++,
            decisionBase,
            INPUT_SCALES[0],
            previousDecisionBase,
            -INPUT_SCALES[0],
            maxDeltaSpeed + f_prevVRefPathMps - f_vRefPathMps
        );
        addConstraint(
            f_constraintCount++,
            decisionBase,
            -INPUT_SCALES[0],
            previousDecisionBase,
            INPUT_SCALES[0],
            maxDeltaSpeed - f_prevVRefPathMps + f_vRefPathMps
        );

        addConstraint(
            f_constraintCount++,
            decisionBase + 1U,
            INPUT_SCALES[1],
            previousDecisionBase + 1U,
            -INPUT_SCALES[1],
            maxDeltaSteer + f_prevDeltaFfRad - f_deltaFfRad
        );
        addConstraint(
            f_constraintCount++,
            decisionBase + 1U,
            -INPUT_SCALES[1],
            previousDecisionBase + 1U,
            INPUT_SCALES[1],
            maxDeltaSteer - f_prevDeltaFfRad + f_deltaFfRad
        );
    }

    void CMpcController::addConstraint(size_t f_rowIndex, size_t f_decisionIndexA, float f_coefficientA, float f_bound)
    {
        for (size_t column = 0U; column < c_decisionCount; column++)
        {
            m_constraintMatrix[f_rowIndex][column] = 0.0f;
        }

        m_constraintMatrix[f_rowIndex][f_decisionIndexA] = f_coefficientA;
        m_constraintBounds[f_rowIndex] = f_bound;
    }

    void CMpcController::addConstraint(size_t f_rowIndex, size_t f_decisionIndexA, float f_coefficientA, size_t f_decisionIndexB, float f_coefficientB, float f_bound)
    {
        for (size_t column = 0U; column < c_decisionCount; column++)
        {
            m_constraintMatrix[f_rowIndex][column] = 0.0f;
        }

        m_constraintMatrix[f_rowIndex][f_decisionIndexA] = f_coefficientA;
        m_constraintMatrix[f_rowIndex][f_decisionIndexB] = f_coefficientB;
        m_constraintBounds[f_rowIndex] = f_bound;
    }

    void CMpcController::projectConstraints(float (&f_solution)[c_decisionCount], size_t f_constraintCount) const
    {
        for (uint8_t sweep = 0U; sweep < PROJECTION_SWEEPS; sweep++)
        {
            float maxViolation = 0.0f;

            for (size_t row = 0U; row < f_constraintCount; row++)
            {
                float dotProduct = 0.0f;
                float normSquared = 0.0f;
                for (size_t column = 0U; column < c_decisionCount; column++)
                {
                    const float coefficient = m_constraintMatrix[row][column];
                    dotProduct += coefficient * f_solution[column];
                    normSquared += coefficient * coefficient;
                }

                const float violation = dotProduct - m_constraintBounds[row];
                if (violation > maxViolation)
                {
                    maxViolation = violation;
                }
                if (violation <= 0.0f || normSquared <= 1e-8f)
                {
                    continue;
                }

                const float scale = violation / normSquared;
                for (size_t column = 0U; column < c_decisionCount; column++)
                {
                    f_solution[column] -= scale * m_constraintMatrix[row][column];
                }
            }

            if (maxViolation <= CONSTRAINT_TOLERANCE)
            {
                break;
            }
        }
    }

    float CMpcController::computeMaxViolation(const float (&f_solution)[c_decisionCount], size_t f_constraintCount) const
    {
        float maxViolation = 0.0f;
        for (size_t row = 0U; row < f_constraintCount; row++)
        {
            float dotProduct = 0.0f;
            for (size_t column = 0U; column < c_decisionCount; column++)
            {
                dotProduct += m_constraintMatrix[row][column] * f_solution[column];
            }

            const float violation = dotProduct - m_constraintBounds[row];
            if (violation > maxViolation)
            {
                maxViolation = violation;
            }
        }

        return maxViolation;
    }

    CMpcController::SolverStatus CMpcController::solveProjectedGradient(
        float (&f_solution)[c_decisionCount],
        size_t f_constraintCount,
        float f_stepSize,
        uint16_t& f_iterations,
        float& f_maxViolation
    )
    {
        float candidate[c_decisionCount];

        for (uint16_t iteration = 0U; iteration < c_maxIterations; iteration++)
        {
            for (size_t row = 0U; row < c_decisionCount; row++)
            {
                float gradientValue = m_gradient[row];
                for (size_t column = 0U; column < c_decisionCount; column++)
                {
                    gradientValue += m_hessian[row][column] * f_solution[column];
                }

                if (!isFiniteFloat(gradientValue))
                {
                    f_iterations = iteration;
                    f_maxViolation = computeMaxViolation(f_solution, f_constraintCount);
                    return SolverStatus::NumericalFailure;
                }

                candidate[row] = f_solution[row] - (f_stepSize * gradientValue);
            }

            projectConstraints(candidate, f_constraintCount);

            float maxUpdate = 0.0f;
            for (size_t index = 0U; index < c_decisionCount; index++)
            {
                const float updateMagnitude = fabsf(candidate[index] - f_solution[index]);
                if (updateMagnitude > maxUpdate)
                {
                    maxUpdate = updateMagnitude;
                }
                f_solution[index] = candidate[index];
            }

            f_maxViolation = computeMaxViolation(f_solution, f_constraintCount);
            if (!isFiniteFloat(f_maxViolation))
            {
                return SolverStatus::NumericalFailure;
            }
            f_iterations = static_cast<uint16_t>(iteration + 1U);
            if ((maxUpdate <= UPDATE_TOLERANCE) && (f_maxViolation <= CONSTRAINT_TOLERANCE))
            {
                return SolverStatus::Success;
            }
        }

        f_maxViolation = computeMaxViolation(f_solution, f_constraintCount);
        return isFiniteFloat(f_maxViolation) ? SolverStatus::IterationLimit : SolverStatus::NumericalFailure;
    }

    float CMpcController::computeObjective(const float (&f_solution)[c_decisionCount]) const
    {
        float quadratic = 0.0f;
        float linear = 0.0f;

        for (size_t row = 0U; row < c_decisionCount; row++)
        {
            float hessianVector = 0.0f;
            for (size_t column = 0U; column < c_decisionCount; column++)
            {
                hessianVector += m_hessian[row][column] * f_solution[column];
            }

            quadratic += f_solution[row] * hessianVector;
            linear += m_gradient[row] * f_solution[row];
        }

        return (0.5f * quadratic) + linear;
    }

    void CMpcController::updatePreviousCorrection(float f_vCorrMps, float f_deltaCorrRad)
    {
        m_previousCorrectionNormalized[0] = f_vCorrMps / INPUT_SCALES[0];
        m_previousCorrectionNormalized[1] = f_deltaCorrRad / INPUT_SCALES[1];
        m_hasPreviousCorrection = true;
    }

    CMpcController::Output CMpcController::makeOutput(
        const Input& f_input,
        SolverStatus f_status,
        bool f_usedPreviousCorrection,
        bool f_usedFeedforwardOnly,
        const float (&f_state)[c_stateCount],
        float f_vCorrMps,
        float f_deltaCorrRad,
        uint16_t f_iterations,
        float f_objective,
        float f_maxViolation
    )
    {
        Output output;

        const float currentVRefPathMps = fabsf(f_input.current.v_body_mps);
        const float unclampedPathSpeedCmd = currentVRefPathMps + f_vCorrMps;
        const float unclampedSteerCmd = f_input.current.delta_ff_rad + f_deltaCorrRad;
        const float pathSpeedCmd = clampFloat(
            unclampedPathSpeedCmd,
            f_input.limits.path_speed_min_mps,
            f_input.limits.path_speed_max_mps
        );
        const float steerCmd = clampFloat(
            unclampedSteerCmd,
            f_input.limits.steer_min_rad,
            f_input.limits.steer_max_rad
        );
        const float maxDeltaSpeed = c_sampleTimeS * f_input.limits.speed_rate_max_mps2;
        const float maxDeltaSteer = c_sampleTimeS * f_input.limits.steer_rate_max_rad_s;
        const float desiredSpeedStep = unclampedPathSpeedCmd - f_input.prev_path_speed_cmd_mps;
        const float desiredSteerStep = unclampedSteerCmd - f_input.prev_steer_cmd_rad;
        const float limitedSpeedStep = clampFloat(desiredSpeedStep, -maxDeltaSpeed, maxDeltaSpeed);
        const float limitedSteerStep = clampFloat(desiredSteerStep, -maxDeltaSteer, maxDeltaSteer);

        output.v_corr_mps = pathSpeedCmd - currentVRefPathMps;
        output.delta_corr_rad = steerCmd - f_input.current.delta_ff_rad;
        output.path_speed_cmd_mps = pathSpeedCmd;
        output.body_speed_cmd_mps = static_cast<float>(f_input.direction) * pathSpeedCmd;
        output.steer_cmd_rad = steerCmd;
        output.e_s_m = f_state[0];
        output.e_y_m = f_state[1];
        output.e_psi_rad = f_state[2];
        output.objective = f_objective;
        output.max_constraint_violation = f_maxViolation;
        output.iterations = f_iterations;
        output.status = f_status;
        output.used_previous_correction = f_usedPreviousCorrection;
        output.used_feedforward_only = f_usedFeedforwardOnly;
        output.speed_rate_limited = (fabsf(limitedSpeedStep - desiredSpeedStep) > 1e-4f);
        output.steer_rate_limited = (fabsf(limitedSteerStep - desiredSteerStep) > 1e-4f);
        output.success = (f_status == SolverStatus::Success);
        return output;
    }
}
