import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray
from ipdb import set_trace as st
from google import genai
import os, time
import json
import pydantic
from typing import List, Optional, Dict, Any, Union
import math
import glob
import argparse

# Import configuration
try:
    from config import GEMINI_API_KEY, DEFAULT_NAMESPACE, DEFAULT_WAIT_TIME, DEFAULT_REPEAT_INTERVAL, GEMINI_MODEL, DEFAULT_CONTEXT_BASE_PATH
except ImportError:
    # Fallback values if config.py is not found
    print("Warning: config.py not found, using default values")
    from config_template import GEMINI_API_KEY, DEFAULT_NAMESPACE, DEFAULT_WAIT_TIME, DEFAULT_REPEAT_INTERVAL, GEMINI_MODEL, DEFAULT_CONTEXT_BASE_PATH

"""
This node listens to a query and generates waypoints for navigation.

Usage:
python3 navigator_llm_node.py --query "chair" --context-file /path/to/context.txt
python3 navigator_llm_node.py --query "plan a path that goes to each of the chairs"
"""

# Define Pydantic models for structured output
class Waypoint(pydantic.BaseModel):
    """A single waypoint with position and orientation"""
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    description: Optional[str] = None

class ObjectWaypoint(pydantic.BaseModel):
    """A waypoint associated with an object"""
    object_name: str
    waypoint: Waypoint
    object_id: Optional[str] = None

class PathPlan(pydantic.BaseModel):
    """A plan with multiple waypoints"""
    waypoints: List[Waypoint]
    description: Optional[str] = None

class ObjectPathPlan(pydantic.BaseModel):
    """A plan with multiple object waypoints"""
    object_waypoints: List[ObjectWaypoint]
    description: Optional[str] = None

class LlmNavigatorNode(Node):
    def __init__(self, namespace: str = "/mobility_gen", context_file: str = "", query: str = ""):
        super().__init__("llm_navigator_node")

        self.namespace = namespace
        self.query = query
        
        # Create publishers for waypoints (like waypoint_publisher.py)
        self.waypoints_pub = self.create_publisher(
            String, f"{self.namespace}/waypoints", 10
        )
        
        self.single_waypoint_pub = self.create_publisher(
            PoseStamped, f"{self.namespace}/waypoint", 10
        )
        
        # Create a publisher for the raw JSON response
        self.response_pub = self.create_publisher(String, '/llm_response', 10)
        
        # Create a publisher for waypoint markers
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        # Marker ID counter
        self.marker_id = 0
        
        # Configure Gemini
        self.client = genai.Client(api_key=GEMINI_API_KEY)

        self.context_file = context_file

        self.context_text = ""
        if self.context_file:
            try:
                with open(self.context_file, 'r') as f:
                    self.context_text = f.read()
                self.get_logger().info(f"Context file '{self.context_file}' loaded.")
            except FileNotFoundError:
                self.get_logger().warn(f"Context file '{self.context_file}' not found.")
            except Exception as e:
                self.get_logger().warn(f"Error loading context file: {e}")

        # Define the base prompt
        self.base_prompt = """
        You are a navigation assistant that helps plan paths to objects in a scene.
        
        The context includes multiple USD files from recent observations, ordered from oldest to newest.
        Each file represents a snapshot of the scene at a specific point in time.
        
        Your task is to analyze these files and generate waypoints for navigation based on the query.
        
        For each waypoint, provide:
        - Position (x, y, z coordinates)
        - Orientation (quaternion: qx, qy, qz, qw)
        - A brief description of the waypoint
        
        If the query is about a specific object, identify that object in the scene and generate a waypoint to it.
        If the query is about planning a path to multiple objects, generate a sequence of waypoints.
        
        You MUST return your response in valid JSON format that matches the provided schema.
        """

        # Define custom schemas for Gemini API (without default values)
        self.waypoint_schema = {
            "type": "object",
            "properties": {
                "x": {"type": "number"},
                "y": {"type": "number"},
                "z": {"type": "number"},
                "qx": {"type": "number"},
                "qy": {"type": "number"},
                "qz": {"type": "number"},
                "qw": {"type": "number"},
                "description": {"type": "string"}
            },
            "required": ["x", "y", "z", "qx", "qy", "qz", "qw"]
        }
        
        self.object_waypoint_schema = {
            "type": "object",
            "properties": {
                "object_name": {"type": "string"},
                "waypoint": self.waypoint_schema,
                "object_id": {"type": "string"}
            },
            "required": ["object_name", "waypoint"]
        }
        
        self.path_plan_schema = {
            "type": "object",
            "properties": {
                "waypoints": {
                    "type": "array",
                    "items": self.waypoint_schema
                },
                "description": {"type": "string"}
            },
            "required": ["waypoints"]
        }
        
        self.object_path_plan_schema = {
            "type": "object",
            "properties": {
                "object_waypoints": {
                    "type": "array",
                    "items": self.object_waypoint_schema
                },
                "description": {"type": "string"}
            },
            "required": ["object_waypoints"]
        }
        
        print(f"✓ LLM Navigator node initialized on namespace: {self.namespace}")
        print(f"  Topics:")
        print(f"    - {self.namespace}/waypoints (String - JSON array of waypoints)")
        print(f"    - {self.namespace}/waypoint (PoseStamped - single waypoint)")
        print(f"    - /llm_response (String - raw JSON response)")
        print(f"    - /waypoint_markers (MarkerArray - visualization markers)")

    def publish_waypoints(self, waypoints):
        """Publish a list of waypoints as JSON string (like waypoint_publisher.py)."""
        try:
            # Convert waypoints to the format expected by MobilityGen
            waypoint_list = []
            for wp in waypoints:
                waypoint_dict = {
                    "x": float(wp.x),
                    "y": float(wp.y),
                    "z": float(wp.z),
                    "qx": float(wp.qx),
                    "qy": float(wp.qy),
                    "qz": float(wp.qz),
                    "qw": float(wp.qw)
                }
                waypoint_list.append(waypoint_dict)
            
            msg = String()
            msg.data = json.dumps({"waypoints": waypoint_list})
            self.waypoints_pub.publish(msg)
            self.get_logger().info(f"✓ Published {len(waypoints)} waypoints to {self.namespace}/waypoints")
            return True
        except Exception as e:
            self.get_logger().error(f"Error publishing waypoints: {e}")
            return False
    
    def publish_single_waypoint(self, waypoint):
        """Publish a single waypoint as PoseStamped (like waypoint_publisher.py)."""
        try:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.pose.position.x = float(waypoint.x)
            msg.pose.position.y = float(waypoint.y)
            msg.pose.position.z = float(waypoint.z)
            
            msg.pose.orientation.x = float(waypoint.qx)
            msg.pose.orientation.y = float(waypoint.qy)
            msg.pose.orientation.z = float(waypoint.qz)
            msg.pose.orientation.w = float(waypoint.qw)
            
            self.single_waypoint_pub.publish(msg)
            self.get_logger().info(f"✓ Published single waypoint: ({waypoint.x:.2f}, {waypoint.y:.2f}, {waypoint.z:.2f})")
            return True
        except Exception as e:
            self.get_logger().error(f"Error publishing single waypoint: {e}")
            return False

    def publish_waypoint_markers(self, waypoints, is_object_waypoints=False):
        """Publish markers for all waypoints in the path"""
        marker_array = MarkerArray()
        
        # Create markers for each waypoint
        for i, waypoint in enumerate(waypoints):
            # Get the actual waypoint object
            wp = waypoint.waypoint if is_object_waypoints else waypoint
            
            # Create sphere marker for waypoint position
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "map"
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = "waypoints"
            sphere_marker.id = i * 3  # Use consistent IDs
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position = Point(x=float(wp.x), y=float(wp.y), z=float(wp.z))
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.2  # 20cm diameter
            sphere_marker.scale.y = 0.2
            sphere_marker.scale.z = 0.2
            sphere_marker.color.r = 0.0
            sphere_marker.color.g = 1.0
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 0.8
            marker_array.markers.append(sphere_marker)
            
            # Create text marker for waypoint label
            text_marker = Marker()
            text_marker.header = sphere_marker.header
            text_marker.ns = "waypoint_labels"
            text_marker.id = i * 3 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = Point(x=float(wp.x), y=float(wp.y), z=float(wp.z) + 0.3)  # Offset above sphere
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.2  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # Create label text
            if is_object_waypoints:
                label = f"{i+1}: {waypoint.object_name}"
                if waypoint.object_id:
                    label += f" ({waypoint.object_id})"
            else:
                label = f"{i+1}"
                if hasattr(wp, 'description') and wp.description:
                    label += f": {wp.description}"
            
            text_marker.text = label
            marker_array.markers.append(text_marker)
            
            # Create arrow marker for orientation
            arrow_marker = Marker()
            arrow_marker.header = sphere_marker.header
            arrow_marker.ns = "waypoint_orientations"
            arrow_marker.id = i * 3 + 2
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position = Point(x=float(wp.x), y=float(wp.y), z=float(wp.z))
            arrow_marker.pose.orientation.x = float(wp.qx)
            arrow_marker.pose.orientation.y = float(wp.qy)
            arrow_marker.pose.orientation.z = float(wp.qz)
            arrow_marker.pose.orientation.w = float(wp.qw)
            arrow_marker.scale.x = 0.3  # Shaft length
            arrow_marker.scale.y = 0.05  # Shaft diameter
            arrow_marker.scale.z = 0.05  # Head diameter
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 0.0
            arrow_marker.color.b = 1.0
            arrow_marker.color.a = 0.8
            marker_array.markers.append(arrow_marker)
        
        # Add a clear marker at the start
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.insert(0, clear_marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(waypoints)} waypoint markers")
        
        # Debug print the first marker
        if marker_array.markers:
            self.get_logger().info(f"First marker: type={marker_array.markers[1].type}, action={marker_array.markers[1].action}, pos=({marker_array.markers[1].pose.position.x}, {marker_array.markers[1].pose.position.y}, {marker_array.markers[1].pose.position.z})")

    def process_query(self, query: str):
        """Process a query and generate waypoints"""
        self.get_logger().info(f"Processing query: {query}")

        # load the usda data based on the date saved by usd_builder.py
        if not self.context_file:
            base_path = DEFAULT_CONTEXT_BASE_PATH
            current_date = time.strftime("%Y%m%d")
            
            # Find all state files for today
            file_pattern = os.path.join(base_path, f"state_{current_date}_*.usda")
            state_files = glob.glob(file_pattern)
            
            # Sort files by number (oldest first) to show progression over time
            state_files.sort(key=lambda x: int(x.split('_')[-1].split('.')[0]))
            
            if state_files:
                try:
                    # Use the most recent file only
                    most_recent_file = state_files[-1]
                    with open(most_recent_file, 'r') as f:
                        self.context_text = f.read()
                    
                    self.get_logger().info(f"Loaded context from most recent file: {os.path.basename(most_recent_file)}")
                except Exception as e:
                    self.get_logger().warn(f"Error loading context file: {e}")
            else:
                self.get_logger().warn(f"No state files found for today ({current_date})")
        
        # Determine which schema to use based on the query
        if "path" in query.lower() or "plan" in query.lower():
            if "each" in query.lower() or "multiple" in query.lower():
                schema_json = json.dumps(self.object_path_plan_schema, indent=2)
                self.current_schema_type = "object_path_plan"
            else:
                schema_json = json.dumps(self.path_plan_schema, indent=2)
                self.current_schema_type = "path_plan"
        else:
            if "each" in query.lower() or "multiple" in query.lower():
                schema_json = json.dumps(self.object_waypoint_schema, indent=2)
                self.current_schema_type = "object_waypoint"
            else:
                schema_json = json.dumps(self.waypoint_schema, indent=2)
                self.current_schema_type = "waypoint"
        
        # Create the full prompt with schema
        full_prompt = f"{self.base_prompt}\n\nUse this JSON schema:\n{schema_json}\n\nQuery: {query}"
        
        # Generate content with the schema
        try:
            self.get_logger().info("Sending request to Gemini API...")
            # Configure generation with schema
            response = self.client.models.generate_content(
                model=GEMINI_MODEL,
                contents=[self.context_text, full_prompt],
                config={
                    'response_mime_type': 'application/json',
                    'response_schema': json.loads(schema_json)
                }
            )
            
            # Extract the response text
            response_text = response.text
            self.get_logger().info(f"Received response from Gemini API: {response_text[:200]}...")
            
            # Try to parse the response as JSON
            try:
                # Find JSON in the response (in case there's additional text)
                json_start = response_text.find('{')
                json_end = response_text.rfind('}') + 1
                
                if json_start >= 0 and json_end > json_start:
                    json_str = response_text[json_start:json_end]
                    self.get_logger().info(f"Extracted JSON: {json_str[:200]}...")
                    parsed_json = json.loads(json_str)
                    
                    # Validate against the schema
                    self.get_logger().info("Validating against schema...")
                    
                    if self.current_schema_type == "waypoint":
                        validated = Waypoint.model_validate(parsed_json)
                        self.publish_waypoint_markers([validated])
                        self.publish_single_waypoint(validated)
                    elif self.current_schema_type == "object_waypoint":
                        validated = ObjectWaypoint.model_validate(parsed_json)
                        self.publish_waypoint_markers([validated], is_object_waypoints=True)
                        self.publish_single_waypoint(validated.waypoint)
                    elif self.current_schema_type == "path_plan":
                        validated = PathPlan.model_validate(parsed_json)
                        self.publish_waypoint_markers(validated.waypoints)
                        self.publish_waypoints(validated.waypoints)
                    elif self.current_schema_type == "object_path_plan":
                        validated = ObjectPathPlan.model_validate(parsed_json)
                        self.publish_waypoint_markers(validated.object_waypoints, is_object_waypoints=True)
                        # Extract waypoints from object waypoints
                        waypoints = [ow.waypoint for ow in validated.object_waypoints]
                        self.publish_waypoints(waypoints)
                    
                    self.get_logger().info(f'Validated response: {json.dumps(parsed_json, indent=2)}')
                    
                    # Publish the validated response
                    self.publish_response(parsed_json)
                else:
                    self.get_logger().warn(f'No JSON found in response: {response_text}')
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Failed to parse JSON: {e}')
                self.get_logger().error(f'Response text: {response_text}')
            except pydantic.ValidationError as e:
                self.get_logger().error(f'Schema validation failed: {e}')
                self.get_logger().error(f'Parsed JSON: {parsed_json}')
        except Exception as e:
            self.get_logger().error(f'Error generating content: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
    
    def publish_response(self, response_data):
        """Publish the raw JSON response"""
        response_msg = String()
        response_msg.data = json.dumps(response_data)
        self.response_pub.publish(response_msg)
        self.get_logger().info(f'Published response: {response_msg.data[:200]}...')


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROS2 LLM Navigator for MobilityGen')
    parser.add_argument('--namespace', default=DEFAULT_NAMESPACE, help='ROS2 namespace')
    parser.add_argument('--context-file', type=str, default='', help='Context file path')
    parser.add_argument('--query', type=str, required=True, help='Navigation query')
    parser.add_argument('--wait', type=float, default=DEFAULT_WAIT_TIME, help='Wait time after publishing (seconds)')
    parser.add_argument('--repeat', action='store_true', help='Repeat waypoints continuously')
    parser.add_argument('--interval', type=float, default=DEFAULT_REPEAT_INTERVAL, help='Interval between repeats (seconds)')
    
    args = parser.parse_args()
    
    navigator = LlmNavigatorNode(
        namespace=args.namespace,
        context_file=args.context_file,
        query=args.query
    )
    
    try:
        # Process the query immediately
        navigator.process_query(args.query)
        
        # Wait a bit to ensure messages are published
        time.sleep(args.wait)
        
        print(f"✓ Query processed and waypoints published to {args.namespace}/waypoints")
        print(f"✓ Raw response published to /llm_response")
        print(f"✓ Visualization markers published to /waypoint_markers")
        
        # Keep the node running like waypoint_publisher
        print(f"✓ Navigator node is running. Press Ctrl+C to stop.")
        print(f"  Topics:")
        print(f"    - {args.namespace}/waypoints (String - JSON array of waypoints)")
        print(f"    - {args.namespace}/waypoint (PoseStamped - single waypoint)")
        print(f"    - /llm_response (String - raw JSON response)")
        print(f"    - /waypoint_markers (MarkerArray - visualization markers)")
        
        if args.repeat:
            print(f"✓ Repeating waypoints every {args.interval} seconds...")
            while rclpy.ok():
                time.sleep(args.interval)
                navigator.process_query(args.query)
        else:
            # Wait a bit to ensure message is published
            time.sleep(1.0)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
