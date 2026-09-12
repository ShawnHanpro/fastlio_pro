#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class ModeManager : public rclcpp::Node {
public:
    ModeManager() : Node("mode_manager") {
        srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/system_mode",
            std::bind(&ModeManager::callback, this, std::placeholders::_1,
                      std::placeholders::_2));

        client_fastlio_ =
            this->create_client<std_srvs::srv::SetBool>("/fastlio/set_mode");

        client_open3d_ =
            this->create_client<std_srvs::srv::SetBool>("/open3d/set_mode");

        client_global_relocalization_ =
            this->create_client<std_srvs::srv::Trigger>(
                "/global_relocalization");

        RCLCPP_INFO(this->get_logger(), "mode manager started");

        // 启动后自动进入定位流程，并触发一次全局重定位
        startup_timer_ =
            this->create_wall_timer(std::chrono::seconds(3), [this]() {
                startup_timer_->cancel();

                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();

                auto res = std::make_shared<std_srvs::srv::SetBool::Response>();

                req->data = true;  // LOCALIZATION

                callback(req, res);

                RCLCPP_INFO(this->get_logger(),
                            "Startup LOCALIZATION requested");
            });
    }

private:
    void callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                  std_srvs::srv::SetBool::Response::SharedPtr      res) {
        auto msg = std::make_shared<std_srvs::srv::SetBool::Request>();

        msg->data = req->data;

        // ========================================================
        // 检查服务
        // ========================================================

        if (!client_fastlio_->wait_for_service(std::chrono::seconds(1))) {
            res->success = false;
            res->message = "/fastlio/set_mode unavailable";

            return;
        }

        if (!client_open3d_->wait_for_service(std::chrono::seconds(1))) {
            res->success = false;
            res->message = "/open3d/set_mode unavailable";

            return;
        }

        // ========================================================
        // MAPPING
        // ========================================================

        if (!req->data) {
            client_open3d_->async_send_request(msg);
            client_fastlio_->async_send_request(msg);

            RCLCPP_INFO(this->get_logger(), "Switch MAPPING");

            res->success = true;
            res->message = "Switching to MAPPING";

            return;
        }

        // ========================================================
        // LOCALIZATION
        // ========================================================

        client_fastlio_->async_send_request(msg);

        client_open3d_->async_send_request(
            msg,
            [this](
                rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                auto response = future.get();

                if (!response->success) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Open3D switch LOCALIZATION failed: %s",
                                 response->message.c_str());
                    return;
                }

                RCLCPP_INFO(this->get_logger(),
                            "Open3D map loaded, trigger global relocalization");

                if (!client_global_relocalization_->wait_for_service(
                        std::chrono::seconds(2))) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "/global_relocalization unavailable");
                    return;
                }

                auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

                client_global_relocalization_->async_send_request(req);
            });

        RCLCPP_INFO(this->get_logger(), "Switch LOCALIZATION");

        res->success = true;
        res->message = "Switching to LOCALIZATION";
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_fastlio_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_open3d_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
        client_global_relocalization_;

    rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ModeManager>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}