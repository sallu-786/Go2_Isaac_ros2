import gradio as gr
import threading
import rclpy
from web_utils.data_stream import DataStream
from web_utils.ros2_topics import (
    view_list_topics, get_topic_operations, execute_topic_operation,build_ros_graph_snapshot)
from web_utils.ros2_services import (
    view_list_services, get_service_operations, execute_service_operation,build_service_graph_snapshot)

from config import SHOW_CAMERA, SHOW_TOPICS,SHOW_SERVICES, SHOW_CONTROLLER, SHOW_DESCRIPTION
from config import CAMERA_TOPIC_NAME, CMD_VEL_PUB_TOPIC_NAME


# ---------- Gradio UI ----------
def main():

        # --- ROS2 Init ---
    rclpy.init(args=None)
    launcher = DataStream(CAMERA_TOPIC_NAME,CMD_VEL_PUB_TOPIC_NAME)

    ros_thread = threading.Thread(target=rclpy.spin, args=(launcher.image_subscriber,), daemon=True)
    desc_thread = threading.Thread(target=launcher.update_description, daemon=True)
    pub_thread = threading.Thread(target=launcher.publish_commands, daemon=True)


    pub_thread.start()
    ros_thread.start()
    if SHOW_DESCRIPTION:      #show LLm description if set as True in config.py
        desc_thread.start()


    # Spin ROS Camera Image subscriber in background -----------------------

    with gr.Blocks(title="MCP Web") as demo:
        gr.Markdown("## 🤖 ROS2-MCP Portal")

        if SHOW_CAMERA:
            

            with gr.Tab("Live Stream"):
                
                live_image = gr.Image(type="numpy", label="Camera View")
                live_text = gr.Textbox(label="Description")

                def live_feed_wrapper():
                    yield from launcher.live_cam_feed("yolo_detection")  # or "color_image"

                # Launch the stream immediately when the app starts
                demo.load(
                    fn=live_feed_wrapper,
                    inputs=None,
                    outputs=[live_image, live_text]
                )
                    # --- Controller Layout ---
                
                if SHOW_CONTROLLER:

                    feedback_box = gr.Textbox(label="Controller Output")
                    with gr.Row():

                        # Left column: rotate left + left
                        with gr.Column():
                            pass
                        with gr.Column():
                            rotate_left_btn = gr.Button("⟲ Left", elem_id="rotate_left")
                            left_btn = gr.Button("← Left", elem_id="left")

                        # Center column: forward / stop / backward
                        with gr.Column():
                            forward_btn = gr.Button("↑ Forward", elem_id="forward")
                            stop_btn = gr.Button("■ Stop", elem_id="stop")
                            backward_btn = gr.Button("↓ Backward", elem_id="backward")

                        # Right column: rotate right + right
                        with gr.Column():
                            rotate_right_btn = gr.Button("⟳ Right", elem_id="rotate_right")
                            right_btn = gr.Button("→ Right", elem_id="right")
                        
                        with gr.Column():
                            pass


                    forward_btn.click(launcher.controller.move_forward,inputs=[],outputs=[feedback_box])
                    backward_btn.click(launcher.controller.move_backward,inputs=[],outputs=[feedback_box])
                    left_btn.click(launcher.controller.move_left,inputs=[],outputs=[feedback_box])
                    right_btn.click(launcher.controller.move_right,inputs=[],outputs=[feedback_box])
                    rotate_left_btn.click(launcher.controller.rotate_left,inputs=[],outputs=[feedback_box])
                    rotate_right_btn.click(launcher.controller.rotate_right,inputs=[],outputs=[feedback_box])
                    stop_btn.click(launcher.controller.stop_robot,inputs=[],outputs=[feedback_box])
                    gr.HTML("""
                        <style>
                        /* Center columns horizontally with small gap */
                        .gr-row.controller-row {
                            display: flex;
                            justify-content: center;  /* center the columns */
                            gap: 1vw;                 /* small gap between columns */
                            flex-wrap: nowrap;         /* prevent wrapping to next line */
                        }

                        /* Responsive buttons */
                        #forward, #backward, #stop, #left, #right, #rotate_left, #rotate_right {
                            width: 10vw;   /* scale with screen width */
                            height: 6vh;   /* scale with screen height */
                            font-size: 1.2vw; 
                            margin: 0.2vw;
                            border-radius: 8px;
                            font-weight: bold;
                            cursor: pointer;
                        }
                        #forward { background-color: #4CAF50; color: white; }
                        #backward { background-color: #2196F3; color: white; }
                        #stop { background-color: #f44336; color: white; }
                        #left, #right { background-color: #e7e7e7; color: black; }
                        #rotate_left, #rotate_right { background-color: #ff9800; color: white; }
                        </style>
                        """)

        # ------------------ TOPICS BLOCK ------------------
        if SHOW_TOPICS:
            with gr.Tab("Topics"):
                with gr.Row():  # create two side-by-side columns

                    # ---------- LEFT COLUMN: Topics ----------
                    with gr.Column():
                        gr.Markdown("### Topics")
                        topic_dropdown = gr.Dropdown(
                            choices=view_list_topics(),
                            label="Select a Topic",
                            multiselect=False
                        )
                        topic_op_dropdown = gr.Dropdown(
                            choices=get_topic_operations(),
                            label="Select Operation",
                            multiselect=False
                        )
                        topic_output = gr.Textbox(label="Topic Result", lines=25)

                        topic_dropdown.change(
                            fn=execute_topic_operation,
                            inputs=[topic_dropdown, topic_op_dropdown],
                            outputs=topic_output
                        )
                        topic_op_dropdown.change(
                            fn=execute_topic_operation,
                            inputs=[topic_dropdown, topic_op_dropdown],
                            outputs=topic_output
                        )

                    # ---------- RIGHT COLUMN: Topics Visualizer ----------
                    with gr.Column():
                        gr.Markdown("### Topics Visualizer")
                        graph_image = gr.Image(type="filepath", label="Topics Graph Snapshot")
                        all_topics = view_list_topics()

                        topic_select = gr.Dropdown(
                            choices=all_topics,
                            label="Select Topics for Graph",
                            value=all_topics,
                            multiselect=True  # allow multiple topics
                        )
                        refresh_btn = gr.Button("Build Graph")

                        def update_selected_graph(selected_topics):
                            return build_ros_graph_snapshot(selected_topics)

                        refresh_btn.click(
                            fn=update_selected_graph,
                            inputs=[topic_select],
                            outputs=graph_image
                        )

        # ------------------ SERVICES BLOCK ------------------
        if SHOW_SERVICES:

            with gr.Tab("Services"):
                with gr.Row():  # create two side-by-side columns

                    # ---------- LEFT COLUMN: Services ----------
                    with gr.Column():
                        gr.Markdown("### Services")
                        service_dropdown = gr.Dropdown(
                            choices=view_list_services(),
                            label="Select a Service",
                            multiselect=False
                        )
                        service_op_dropdown = gr.Dropdown(
                            choices=get_service_operations(),
                            label="Select Operation",
                            multiselect=False
                        )
                        service_output = gr.Textbox(label="Service Result", lines=25)

                        service_dropdown.change(
                            fn=execute_service_operation,
                            inputs=[service_dropdown, service_op_dropdown],
                            outputs=service_output
                        )
                        service_op_dropdown.change(
                            fn=execute_service_operation,
                            inputs=[service_dropdown, service_op_dropdown],
                            outputs=service_output
                        )

                    # ---------- RIGHT COLUMN: Services Visualizer ----------

                    with gr.Column():
                        gr.Markdown("### Services Visualizer")
                        graph_image = gr.Image(type="filepath", label="Topics Graph Snapshot")
                        all_topics = view_list_services() 
                        topic_select = gr.Dropdown(
                            choices=all_topics,
                            label="Select Topics for Graph",
                            value=all_topics,
                            multiselect=True  # allow multiple services
                        )
                        refresh_btn = gr.Button("Build Graph")

                        def update_selected_graph(selected_topics):
                            return build_service_graph_snapshot(selected_topics)

                        refresh_btn.click(
                            fn=update_selected_graph,
                            inputs=[topic_select],
                            outputs=graph_image
                        )

        demo.launch(server_name = "0.0.0.0", 
                    server_port = 7860,
                    pwa=True, 
                    share=False, #sharing will make a cloud link that allows access over the internet
                    mcp_server=True, #web_portal serves as mcp_server as well
                   )

if __name__ == "__main__":
    main()
