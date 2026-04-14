#ifndef MPCCONTROLLER_HPP
#define MPCCONTROLLER_HPP

#include <cstddef>
#include <cstdint>

namespace brain
{
    class CMpcController
    {
        public:
            static constexpr size_t c_stateCount = 3U;
            static constexpr size_t c_inputCount = 2U;
            static constexpr size_t c_horizonLength = 10U;
            static constexpr size_t c_decisionCount = c_inputCount * c_horizonLength;

            static constexpr float c_sampleTimeS = 0.05f;
            static constexpr float c_wheelbaseM = 0.26f;
            static constexpr float c_defaultPathSpeedMaxMps = 0.50f;
            static constexpr float c_defaultSteerLimitRad = 0.436332313f;
            static constexpr float c_defaultSpeedRateLimitMps2 = 0.8f;
            static constexpr float c_defaultSteerRateLimitRadS = 1.04719755f;

            static constexpr float c_eSScaleM = 0.05f;
            static constexpr float c_eYScaleM = 0.02f;
            static constexpr float c_ePsiScaleRad = 0.0872665f;
            static constexpr float c_vCorrScaleMps = 0.05f;
            static constexpr float c_deltaCorrScaleRad = 0.0698132f;

            static constexpr float c_qES = 1.0f;
            static constexpr float c_qEY = 6.0f;
            static constexpr float c_qEPsi = 4.0f;
            static constexpr float c_rV = 0.35f;
            static constexpr float c_rDelta = 0.80f;
            static constexpr float c_sV = 0.25f;
            static constexpr float c_sDelta = 1.40f;
            static constexpr float c_pES = 2.0f;
            static constexpr float c_pEY = 12.0f;
            static constexpr float c_pEPsi = 8.0f;

            enum class SolverStatus : uint8_t
            {
                Disabled = 0U,
                Success,
                InvalidInput,
                NumericalFailure,
                IterationLimit,
                FallbackPreviousCorrection,
                FeedforwardOnly,
                RecoveryRelease
            };

            struct PoseEstimate
            {
                bool valid;
                float x_m;
                float y_m;
                float psi_rad;
            };

            struct ReferenceSample
            {
                float matched_progress_m;
                float nominal_progress_m;
                float x_m;
                float y_m;
                float psi_rad;
                float theta_rad;
                float v_body_mps;
                float delta_ff_rad;
            };

            struct HorizonSample
            {
                float v_body_mps;
                float delta_ff_rad;
            };

            struct Limits
            {
                float path_speed_min_mps;
                float path_speed_max_mps;
                float steer_min_rad;
                float steer_max_rad;
                float speed_rate_max_mps2;
                float steer_rate_max_rad_s;
            };

            struct Input
            {
                bool feedback_enabled;
                bool dynamic_segment;
                int8_t direction;
                PoseEstimate pose;
                ReferenceSample current;
                HorizonSample horizon[c_horizonLength];
                Limits limits;
                float prev_path_speed_cmd_mps;
                float prev_steer_cmd_rad;
            };

            struct Output
            {
                float v_corr_mps;
                float delta_corr_rad;
                float path_speed_cmd_mps;
                float body_speed_cmd_mps;
                float steer_cmd_rad;
                float e_s_m;
                float e_y_m;
                float e_psi_rad;
                float objective;
                float max_constraint_violation;
                uint16_t iterations;
                SolverStatus status;
                bool used_previous_correction;
                bool used_feedforward_only;
                bool success;

                Output();
            };

            CMpcController();

            void reset();
            void resetWarmStart();
            Output solve(const Input& f_input);

        private:
            static constexpr size_t c_maxConstraintCount = (8U * c_horizonLength);
            static constexpr uint16_t c_maxIterations = 48U;

            static float wrapAngle(float f_angle);
            static float clampFloat(float f_value, float f_minValue, float f_maxValue);
            static float moveToward(float f_current, float f_target, float f_maxStep);

            void clearProblem();
            void shiftWarmStart(float (&f_shiftedSolution)[c_decisionCount]) const;
            void clearPreviousCorrection();
            void buildCurrentState(const Input& f_input, float (&f_state)[c_stateCount]) const;
            bool shouldInvalidateForLargeDisturbance(const float (&f_state)[c_stateCount]) const;
            bool buildCondensedProblem(
                const Input& f_input,
                const float (&f_state)[c_stateCount],
                const float (&f_previousCorrectionNormalized)[c_inputCount],
                size_t& f_constraintCount,
                float& f_stepSize
            );
            void appendCommandConstraints(const Input& f_input, size_t f_stageIndex, float f_vRefPathMps, float f_deltaFfRad, size_t& f_constraintCount);
            void appendRateConstraints(const Input& f_input, size_t f_stageIndex, float f_vRefPathMps, float f_deltaFfRad, float f_prevVRefPathMps, float f_prevDeltaFfRad, size_t& f_constraintCount);
            void addConstraint(size_t f_rowIndex, size_t f_decisionIndexA, float f_coefficientA, float f_bound);
            void addConstraint(size_t f_rowIndex, size_t f_decisionIndexA, float f_coefficientA, size_t f_decisionIndexB, float f_coefficientB, float f_bound);
            void projectConstraints(float (&f_solution)[c_decisionCount], size_t f_constraintCount) const;
            float computeMaxViolation(const float (&f_solution)[c_decisionCount], size_t f_constraintCount) const;
            SolverStatus solveProjectedGradient(
                float (&f_solution)[c_decisionCount],
                size_t f_constraintCount,
                float f_stepSize,
                uint16_t& f_iterations,
                float& f_maxViolation
            );
            float computeObjective(const float (&f_solution)[c_decisionCount]) const;
            void updatePreviousCorrection(float f_vCorrMps, float f_deltaCorrRad);
            Output makeOutput(const Input& f_input, SolverStatus f_status, bool f_usedPreviousCorrection, bool f_usedFeedforwardOnly, const float (&f_state)[c_stateCount], float f_vCorrMps, float f_deltaCorrRad, uint16_t f_iterations, float f_objective, float f_maxViolation);

            bool m_hasWarmStart;
            float m_warmStart[c_decisionCount];
            bool m_hasPreviousCorrection;
            float m_previousCorrectionNormalized[c_inputCount];

            float m_hessian[c_decisionCount][c_decisionCount];
            float m_gradient[c_decisionCount];
            float m_constraintMatrix[c_maxConstraintCount][c_decisionCount];
            float m_constraintBounds[c_maxConstraintCount];
    };
}

#endif // MPCCONTROLLER_HPP
