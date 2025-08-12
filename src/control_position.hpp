#pragma once

class PositionController {
	public:
		PositionController(const control_position_cfg_t& cfg) {
            control_position_init(&this->controller, &cfg);
		};

		void update(uint32_t now_us) {
            control_position_update(&this->controller, now_us);
		};

        void reportPos(float pos) {
            control_position_report_pos(&this->controller, pos);
        };

        void targetPos(float pos) {
            control_position_target_pos(&this->controller, pos);
        };

        void setCoastVel(float vel) {
            control_position_set_coast_vel(&this->controller, vel);
        };

        void pauseIf(bool pause) {
            control_position_pause_if(&this->controller, pause);
        };

        void pause() {
            control_position_pause(&this->controller);
        };

        void unpause() {
            control_position_unpause(&this->controller);
        };

        void stop() {
            control_position_stop(&this->controller);
        };

        float stopPos() const {
            return control_position_stop_pos(&this->controller);
        };

        float velOutput() const {
            return this->controller.vel_output;
        };

        bool targetReached() const {
            return this->controller.target_reached;
        };

        bool& targetReached() {
            return this->controller.target_reached;
        };

	public:
		control_position_t     controller;
};
