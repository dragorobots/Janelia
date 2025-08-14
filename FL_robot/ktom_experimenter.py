import numpy as np
from scipy.special import softmax, logsumexp
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import csv
from datetime import datetime
import os
import roslibpy
import threading
import time

# --- Robot Communication Class ---
class RosLink:
    def __init__(self, host, port=10090):
        self.ros = roslibpy.Ros(host=host, port=port)
        self.target = roslibpy.Topic(self.ros,'/hide_and_seek/target_spot','std_msgs/msg/Int32')
        self.toggle = roslibpy.Topic(self.ros,'/hide_and_seek/toggles','std_msgs/msg/String')
        self.cmdvel = roslibpy.Topic(self.ros,'/hide_and_seek/cmd_vel','geometry_msgs/msg/Twist')
        self.lf = roslibpy.Topic(self.ros,'/line_follow/status','std_msgs/msg/String')
        self.rat = roslibpy.Topic(self.ros,'/rat_detection/found','std_msgs/msg/Bool')
        self.prog = roslibpy.Topic(self.ros,'/hide_and_seek/progress','std_msgs/msg/String')
        self.connected = False
        self.status_callbacks = []

    def connect(self, on_lf=None, on_rat=None, on_prog=None):
        try:
            self.ros.run()
            self.target.advertise()
            self.toggle.advertise()
            self.cmdvel.advertise()
            if on_lf: self.lf.subscribe(lambda m: on_lf(m['data']))
            if on_rat: self.rat.subscribe(lambda m: on_rat(bool(m['data'])))
            if on_prog: self.prog.subscribe(lambda m: on_prog(m['data']))
            self.connected = True
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def send_target(self, i):
        if self.connected:
            self.target.publish(roslibpy.Message({'data': int(i)}))

    def send_toggle(self, s):
        if self.connected:
            self.toggle.publish(roslibpy.Message({'data': str(s)}))

    def send_cmdvel(self, v, w):
        if self.connected:
            self.cmdvel.publish(roslibpy.Message({
                'linear': {'x': float(v), 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': float(w)}
            }))

    def add_status_callback(self, callback):
        self.status_callbacks.append(callback)

    def close(self):
        self.connected = False
        for t in [self.target, self.toggle, self.cmdvel, self.lf, self.rat, self.prog]:
            try: t.unadvertise()
            except: pass
        self.ros.terminate()

# --- Core k-ToM Model Implementation (Corrected for Data Flow) ---
class ToMAgent:
    """Represents a Theory of Mind agent, now with corrected data flow."""
    def __init__(self, k_level, num_spots, is_robot=False, robot_history=None):
        self.k = k_level
        self.num_spots = num_spots
        self.is_robot = is_robot
        self.opponent_choice_beliefs = np.ones(num_spots)
        self.virtual_opponents = []

        if self.k > 0:
            if robot_history is None: robot_history = []
            self.robot_history = robot_history
            self.k_levels_modeled = list(range(-1, self.k))
            for k_prime in self.k_levels_modeled:
                self.virtual_opponents.append(
                    ToMAgent(k_prime, num_spots, is_robot=False, robot_history=self.robot_history)
                )
            self.opponent_k_level_beliefs = np.ones(len(self.k_levels_modeled)) / len(self.k_levels_modeled)
        else:
            self.k_levels_modeled = []
            self.opponent_k_level_beliefs = np.array([])

        self.learning_rate = 0.7
        self.beta = 3.0

    def get_choice_probabilities(self):
        """Calculates choice probabilities for a k=0 agent using softmax."""
        expected_probs = self.opponent_choice_beliefs / np.sum(self.opponent_choice_beliefs)
        return softmax(expected_probs * self.beta)

    def predict_opponent_choice_dist(self):
        """Predicts the probability distribution of the opponent's next choice."""
        if self.k == -1:
            return self.opponent_choice_beliefs / np.sum(self.opponent_choice_beliefs)
        if self.k == 0:
            return self.get_choice_probabilities()
        if self.k > 0:
            predictions = np.array([
                vo.predict_opponent_choice_dist() for vo in self.virtual_opponents
            ])
            log_beliefs = np.log(self.opponent_k_level_beliefs + 1e-9)
            final_prediction = np.exp(logsumexp(log_beliefs[:, np.newaxis] + np.log(predictions + 1e-9), axis=0))
            return final_prediction

    # --- MODIFIED: The entire update logic has been rewritten for clarity and correctness ---
    def update_beliefs(self, rat_actual_choice, robot_last_choice):
        """
        Updates agent beliefs. This is the core learning method.
        - A k=-1 model learns from the rat's history (rat_actual_choice).
        - k=0 and k>0 models learn from the robot's history (robot_last_choice),
          as they model an opponent who is reacting to the robot.
        """
        # 1. Update this agent's own internal beliefs based on its k-level.
        if self.k == -1:
            # The bias model learns from the rat's raw choice frequency.
            if rat_actual_choice is not None:
                self.opponent_choice_beliefs[rat_actual_choice] += 1
        elif self.k == 0:
            # The reactive model learns from the robot's moves with decay.
            if robot_last_choice is not None:
                self.opponent_choice_beliefs *= (1 - self.learning_rate)
                self.opponent_choice_beliefs[robot_last_choice] += self.learning_rate
                self.opponent_choice_beliefs = np.maximum(self.opponent_choice_beliefs, 0.01)

        # 2. For the top-level robot, update its beliefs about the rat's k-level.
        if self.is_robot:
            self.robot_history.append(robot_last_choice)
            if self.k > 0 and rat_actual_choice is not None:
                log_likelihoods = np.zeros(len(self.k_levels_modeled))
                for i, vo in enumerate(self.virtual_opponents):
                    prediction_dist = vo.predict_opponent_choice_dist()
                    prob_of_actual_choice = prediction_dist[rat_actual_choice]
                    log_likelihoods[i] = np.log(prob_of_actual_choice + 1e-9)
                log_posteriors = np.log(self.opponent_k_level_beliefs + 1e-9) + log_likelihoods
                self.opponent_k_level_beliefs = softmax(log_posteriors)

        # 3. Recursively update all virtual opponents.
        if self.k > 0:
            for vo in self.virtual_opponents:
                # Pass the same information down the chain. Each model will use
                # the piece of information relevant to its own k-level.
                vo.update_beliefs(rat_actual_choice, robot_last_choice)

# --- Main Controller for the Robot (Corrected) ---
class RobotController:
    """Manages the robot's strategy and logs detailed trial data."""
    def __init__(self):
        # Experiment parameters
        self.k_level = 0
        self.num_spots = 0
        self.k0_strategy = ''
        self.k0_config_str = ''
        # Trial state
        self.trial_num = 0
        self.robot_agent = None
        self.current_recommendation = None
        self.trial_log = []
        self.found_times = []
        self.max_trial_time = 120.0
        # Pre-trial model state for logging
        self.pre_trial_model_state = {}

    def initialize_game(self, k_level, num_spots, k0_strategy, k0_config):
        self.k_level = k_level
        self.num_spots = num_spots
        self.trial_num = 0
        self.robot_choice_history = []
        self.rat_choice_history = []
        self.trial_log = []
        self.found_times = []
        if self.k_level == 0:
            self.k0_strategy = k0_strategy
            if self.k0_strategy == 'patterned':
                self.k0_pattern = k0_config
                self.k0_config_str = str([c+1 for c in k0_config])
            else:
                self.k0_percentages = k0_config
                self.k0_config_str = str(k0_config)
            # k=0 robot doesn't use ToM
            self.robot_agent = None
        else:
            # --- MODIFIED: All adaptive robots now use the enhanced ToM Agent ---
            self.robot_agent = ToMAgent(k_level, num_spots, is_robot=True, robot_history=self.robot_choice_history)

    def recommend_spot(self):
        """Recommends a spot and captures the pre-decision model state for logging."""
        self.trial_num += 1
        # --- MODIFIED: k=0 choice is now separate from k>0 choice ---
        if self.k_level == 0:
            choice = self.k0_pattern[(self.trial_num - 1) % len(self.k0_pattern)] if self.k0_strategy == 'patterned' else np.random.choice(self.num_spots, p=self.k0_percentages)
            self.pre_trial_model_state = {}
        else:
            rat_predicted_dist = self.robot_agent.predict_opponent_choice_dist()

            # Find the safest spots (those with minimum predicted search probability)
            min_probability = np.min(rat_predicted_dist)
            safest_spots = np.where(np.isclose(rat_predicted_dist, min_probability))[0]

            # Randomly choose one of the safest spots to be unpredictable
            choice = np.random.choice(safest_spots)

            self.pre_trial_model_state = {
                'k_beliefs': self.robot_agent.opponent_k_level_beliefs,
                'k_levels': self.robot_agent.k_levels_modeled,
                'rat_search_prediction': rat_predicted_dist
            }

        self.current_recommendation = choice
        return choice

    def process_trial_result(self, rat_choice, was_found, time_taken, search_sequence=""):
        """Updates the k-ToM model and appends a detailed record to the trial log."""
        rat_choice_idx = int(rat_choice)
        robot_hid_spot = self.current_recommendation
        self.rat_choice_history.append(rat_choice_idx)
        self.robot_choice_history.append(robot_hid_spot)
        # --- MODIFIED: Update only happens for k>0 robots ---
        if self.k_level > 0:
            self.robot_agent.update_beliefs(rat_choice_idx, robot_hid_spot)
        if was_found:
            self.found_times.append(time_taken)

        log_entry = {
            "trial_num": self.trial_num,
            "robot_k_level": self.k_level,
            "num_hiding_spots": self.num_spots,
            "robot_hiding_spot": robot_hid_spot + 1,
            "rat_first_search": rat_choice_idx + 1,
            "was_found": was_found,
            "time_to_find": time_taken,
            "search_sequence": search_sequence
        }
        if self.k_level == 0:
            log_entry['k0_strategy'] = self.k0_strategy
            log_entry['k0_configuration'] = self.k0_config_str
        elif self.k_level > 0:
            # Log the robot's beliefs about the rat's k-level
            for i, p_k in enumerate(self.pre_trial_model_state['k_beliefs']):
                k_val = self.pre_trial_model_state['k_levels'][i]
                log_entry[f'belief_rat_is_k{k_val}'] = p_k
            # Log the robot's prediction of the rat's search
            for i, p_search in enumerate(self.pre_trial_model_state['rat_search_prediction']):
                log_entry[f'pred_rat_searches_spot{i+1}'] = p_search

        self.trial_log.append(log_entry)

    def get_model_state_info(self):
        if not self.robot_agent: return "k=0 Robot is non-adaptive. No internal model state."
        info = []
        if self.k_level > 0:
            k_beliefs = self.robot_agent.opponent_k_level_beliefs
            k_levels = self.robot_agent.k_levels_modeled
            # --- MODIFIED: Displays beliefs for all modeled k-levels, including k=-1 ---
            belief_str = ", ".join([f"P(Rat is k={k_val}): {p:.2%}" for k_val, p in zip(k_levels, k_beliefs)])
            info.append(f"Robot's Belief about Rat's Sophistication:\n{belief_str}")

        rat_pred_dist = self.robot_agent.predict_opponent_choice_dist()
        pred_str = ", ".join([f"Spot {i+1}: {p:.2%}" for i, p in enumerate(rat_pred_dist)])
        info.append(f"Robot's Prediction of Rat's Next Search:\n{pred_str}")
        avg_time_str = f"{np.mean(self.found_times):.1f}s" if self.found_times else "N/A"
        info.append(f"Avg. Time if Found: {avg_time_str}")
        return "\n\n".join(info)

class KToMExperimenterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("k-ToM Hide and Seek Experimenter")
        self.root.geometry("1000x900")
        
        self.controller = RobotController()
        self.robot_link = None
        self.robot_status = "Disconnected"
        self.setup_gui()
        
    def setup_gui(self):
        # Create main notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Setup tab
        setup_frame = ttk.Frame(notebook)
        notebook.add(setup_frame, text="Setup")
        self.create_setup_tab(setup_frame)
        
        # Trial tab
        trial_frame = ttk.Frame(notebook)
        notebook.add(trial_frame, text="Trial Execution")
        self.create_trial_tab(trial_frame)
        
        # Robot Control tab
        robot_frame = ttk.Frame(notebook)
        notebook.add(robot_frame, text="Robot Control")
        self.create_robot_control_tab(robot_frame)
        
    def create_setup_tab(self, parent):
        # Title
        title_label = ttk.Label(parent, text="1. Initial Setup", font=('Arial', 16, 'bold'))
        title_label.pack(pady=10)
        
        # Setup frame
        setup_frame = ttk.LabelFrame(parent, text="Experiment Configuration", padding=10)
        setup_frame.pack(fill='x', padx=10, pady=5)
        
        # K-level selection
        ttk.Label(setup_frame, text="Robot's k-ToM Level:").pack(anchor='w')
        self.k_level_var = tk.IntVar(value=2)
        k_level_combo = ttk.Combobox(setup_frame, textvariable=self.k_level_var, 
                                    values=[0, 1, 2, 3], state='readonly')
        k_level_combo.pack(fill='x', pady=2)
        k_level_combo.bind('<<ComboboxSelected>>', self.on_k_level_change)
        
        # Number of spots
        ttk.Label(setup_frame, text="Number of Hiding Spots:").pack(anchor='w', pady=(10,0))
        self.num_spots_var = tk.IntVar(value=4)
        num_spots_spin = ttk.Spinbox(setup_frame, from_=2, to=10, textvariable=self.num_spots_var)
        num_spots_spin.pack(fill='x', pady=2)
        
        # K=0 strategy frame
        self.k0_frame = ttk.LabelFrame(setup_frame, text="k=0 Strategy Configuration")
        self.k0_frame.pack(fill='x', pady=10)
        
        # K=0 strategy type
        self.k0_strategy_var = tk.StringVar(value='Patterned')
        ttk.Radiobutton(self.k0_frame, text="Patterned", variable=self.k0_strategy_var, 
                       value='Patterned', command=self.on_k0_strategy_change).pack(anchor='w')
        ttk.Radiobutton(self.k0_frame, text="Percentage", variable=self.k0_strategy_var, 
                       value='Percentage', command=self.on_k0_strategy_change).pack(anchor='w')
        
        # Pattern input
        self.pattern_frame = ttk.Frame(self.k0_frame)
        self.pattern_frame.pack(fill='x', pady=5)
        ttk.Label(self.pattern_frame, text="Pattern (1-based, comma-separated):").pack(anchor='w')
        self.pattern_var = tk.StringVar(value='1,2,3,4')
        self.pattern_entry = ttk.Entry(self.pattern_frame, textvariable=self.pattern_var)
        self.pattern_entry.pack(fill='x', pady=2)
        
        # Percentage input
        self.percentage_frame = ttk.Frame(self.k0_frame)
        ttk.Label(self.percentage_frame, text="Percentages (comma-separated, must sum to 1.0):").pack(anchor='w')
        self.percentage_var = tk.StringVar(value='0.25,0.25,0.25,0.25')
        self.percentage_entry = ttk.Entry(self.percentage_frame, textvariable=self.percentage_var)
        self.percentage_entry.pack(fill='x', pady=2)
        
        # Start button
        self.start_button = ttk.Button(setup_frame, text="Start Challenge", 
                                      command=self.on_start_button_clicked, style='Accent.TButton')
        self.start_button.pack(pady=10)
        
        # Initialize k0 frame visibility
        self.on_k_level_change(None)
        self.on_k0_strategy_change()
        
    def create_trial_tab(self, parent):
        # Title
        title_label = ttk.Label(parent, text="2. Trial Execution", font=('Arial', 16, 'bold'))
        title_label.pack(pady=10)
        
        # Trial info frame
        trial_info_frame = ttk.LabelFrame(parent, text="Current Trial", padding=10)
        trial_info_frame.pack(fill='x', padx=10, pady=5)
        
        self.trial_num_label = ttk.Label(trial_info_frame, text="Trial #1", font=('Arial', 12, 'bold'))
        self.trial_num_label.pack()
        
        # Recommendation display
        self.recommendation_label = ttk.Label(trial_info_frame, text="Robot Recommends Hiding In Spot: ?", 
                                            font=('Arial', 14, 'bold'), foreground='blue')
        self.recommendation_label.pack(pady=10)
        
        # Trial input frame
        input_frame = ttk.LabelFrame(parent, text="Enter Trial Outcome", padding=10)
        input_frame.pack(fill='x', padx=10, pady=5)
        
        # Rat's first search
        ttk.Label(input_frame, text="Rat's First Search (1-based):").pack(anchor='w')
        self.rat_choice_var = tk.IntVar(value=1)
        self.rat_choice_spin = ttk.Spinbox(input_frame, from_=1, to=10, textvariable=self.rat_choice_var)
        self.rat_choice_spin.pack(fill='x', pady=2)
        self.rat_choice_spin.bind('<KeyRelease>', self.on_rat_choice_change)
        
        # Complete search sequence
        ttk.Label(input_frame, text="Complete Search Sequence (comma-separated, A-D or 1-4):").pack(anchor='w', pady=(10,0))
        self.search_sequence_var = tk.StringVar(value="")
        self.search_sequence_entry = ttk.Entry(input_frame, textvariable=self.search_sequence_var)
        self.search_sequence_entry.pack(fill='x', pady=2)
        ttk.Label(input_frame, text="Example: A,B,C,D or 1,2,3,4 or A,C,B (if found at B)", 
                 font=('Arial', 8), foreground='gray').pack(anchor='w')
        
        # Found checkbox
        self.found_var = tk.BooleanVar(value=False)
        self.found_check = ttk.Checkbutton(input_frame, text="Was the robot found?", 
                                          variable=self.found_var)
        self.found_check.pack(anchor='w', pady=5)
        
        # Time input
        ttk.Label(input_frame, text="Time to find (seconds):").pack(anchor='w')
        self.time_var = tk.DoubleVar(value=120.0)
        self.time_spin = ttk.Spinbox(input_frame, from_=0.0, to=1000.0, increment=0.1, 
                                   textvariable=self.time_var)
        self.time_spin.pack(fill='x', pady=2)
        
        # Submit button
        self.submit_button = ttk.Button(input_frame, text="Submit Trial & Get Next Recommendation", 
                                       command=self.on_submit_button_clicked)
        self.submit_button.pack(pady=10)
        
        # Trial log frame
        log_frame = ttk.LabelFrame(parent, text="Trial Log", padding=10)
        log_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Create text widget with scrollbar for trial log
        log_text_frame = ttk.Frame(log_frame)
        log_text_frame.pack(fill='both', expand=True)
        
        self.trial_log_text = tk.Text(log_text_frame, height=10, wrap='word')
        log_scrollbar = ttk.Scrollbar(log_text_frame, orient='vertical', command=self.trial_log_text.yview)
        self.trial_log_text.configure(yscrollcommand=log_scrollbar.set)
        
        self.trial_log_text.pack(side='left', fill='both', expand=True)
        log_scrollbar.pack(side='right', fill='y')
        
        # Model state frame
        model_frame = ttk.LabelFrame(parent, text="Model State", padding=10)
        model_frame.pack(fill='x', padx=10, pady=5)
        
        self.model_state_text = tk.Text(model_frame, height=6, wrap='word')
        model_scrollbar = ttk.Scrollbar(model_frame, orient='vertical', command=self.model_state_text.yview)
        self.model_state_text.configure(yscrollcommand=model_scrollbar.set)
        
        self.model_state_text.pack(side='left', fill='both', expand=True)
        model_scrollbar.pack(side='right', fill='y')
        
        # Generate log button
        self.generate_log_button = ttk.Button(parent, text="Generate Trial Log (.csv)", 
                                            command=self.on_generate_log_clicked)
        self.generate_log_button.pack(pady=10)
        
    def create_robot_control_tab(self, parent):
        # Title
        title_label = ttk.Label(parent, text="3. Robot Control", font=('Arial', 16, 'bold'))
        title_label.pack(pady=10)
        
        # Connection frame
        connection_frame = ttk.LabelFrame(parent, text="Robot Connection", padding=10)
        connection_frame.pack(fill='x', padx=10, pady=5)
        
        # Robot IP input
        ttk.Label(connection_frame, text="Robot IP Address:").pack(anchor='w')
        self.robot_ip_var = tk.StringVar(value="192.168.1.100")
        self.robot_ip_entry = ttk.Entry(connection_frame, textvariable=self.robot_ip_var)
        self.robot_ip_entry.pack(fill='x', pady=2)
        
        # Connection button
        self.connect_button = ttk.Button(connection_frame, text="Connect to Robot", 
                                        command=self.on_connect_robot)
        self.connect_button.pack(pady=5)
        
        # Status display
        self.robot_status_label = ttk.Label(connection_frame, text="Status: Disconnected", 
                                           foreground='red')
        self.robot_status_label.pack(pady=5)
        
        # Robot Control frame
        control_frame = ttk.LabelFrame(parent, text="Robot Commands", padding=10)
        control_frame.pack(fill='x', padx=10, pady=5)
        
        # Target spot selection
        ttk.Label(control_frame, text="Target Hiding Spot:").pack(anchor='w')
        self.target_spot_var = tk.StringVar(value="A")
        target_combo = ttk.Combobox(control_frame, textvariable=self.target_spot_var, 
                                   values=["A", "B", "C", "D"], state='readonly')
        target_combo.pack(fill='x', pady=2)
        
        # Send target button
        self.send_target_button = ttk.Button(control_frame, text="Send Target Spot", 
                                            command=self.on_send_target)
        self.send_target_button.pack(pady=5)
        
        # Mode toggles
        ttk.Label(control_frame, text="Drive Mode:").pack(anchor='w', pady=(10,0))
        self.drive_mode_var = tk.StringVar(value="auto_line")
        drive_combo = ttk.Combobox(control_frame, textvariable=self.drive_mode_var, 
                                  values=["auto_line", "manual_line", "manual_drive"], state='readonly')
        drive_combo.pack(fill='x', pady=2)
        
        # Send drive mode button
        self.send_drive_mode_button = ttk.Button(control_frame, text="Set Drive Mode", 
                                                command=self.on_send_drive_mode)
        self.send_drive_mode_button.pack(pady=5)
        
        # Rat detection mode
        ttk.Label(control_frame, text="Rat Detection Mode:").pack(anchor='w', pady=(10,0))
        self.rat_mode_var = tk.StringVar(value="auto")
        rat_combo = ttk.Combobox(control_frame, textvariable=self.rat_mode_var, 
                                values=["auto", "manual"], state='readonly')
        rat_combo.pack(fill='x', pady=2)
        
        # Send rat mode button
        self.send_rat_mode_button = ttk.Button(control_frame, text="Set Rat Mode", 
                                              command=self.on_send_rat_mode)
        self.send_rat_mode_button.pack(pady=5)
        
        # Manual found button
        self.manual_found_button = ttk.Button(control_frame, text="Manual: Rat Found!", 
                                             command=self.on_manual_found, style='Accent.TButton')
        self.manual_found_button.pack(pady=10)
        
        # Manual driving frame
        drive_frame = ttk.LabelFrame(parent, text="Manual Driving", padding=10)
        drive_frame.pack(fill='x', padx=10, pady=5)
        
        # Speed controls
        ttk.Label(drive_frame, text="Linear Speed (m/s):").pack(anchor='w')
        self.linear_speed_var = tk.DoubleVar(value=0.2)
        self.linear_speed_spin = ttk.Spinbox(drive_frame, from_=-1.0, to=1.0, increment=0.1, 
                                           textvariable=self.linear_speed_var)
        self.linear_speed_spin.pack(fill='x', pady=2)
        
        ttk.Label(drive_frame, text="Angular Speed (rad/s):").pack(anchor='w')
        self.angular_speed_var = tk.DoubleVar(value=0.0)
        self.angular_speed_spin = ttk.Spinbox(drive_frame, from_=-2.0, to=2.0, increment=0.1, 
                                            textvariable=self.angular_speed_var)
        self.angular_speed_spin.pack(fill='x', pady=2)
        
        # Send velocity button
        self.send_velocity_button = ttk.Button(drive_frame, text="Send Velocity Command", 
                                              command=self.on_send_velocity)
        self.send_velocity_button.pack(pady=5)
        
        # Stop button
        self.stop_button = ttk.Button(drive_frame, text="STOP", 
                                     command=self.on_stop_robot, style='Accent.TButton')
        self.stop_button.pack(pady=5)
        
        # Robot status frame
        status_frame = ttk.LabelFrame(parent, text="Robot Status", padding=10)
        status_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Status text widget
        self.robot_status_text = tk.Text(status_frame, height=8, wrap='word')
        status_scrollbar = ttk.Scrollbar(status_frame, orient='vertical', command=self.robot_status_text.yview)
        self.robot_status_text.configure(yscrollcommand=status_scrollbar.set)
        
        self.robot_status_text.pack(side='left', fill='both', expand=True)
        status_scrollbar.pack(side='right', fill='y')
        
    def on_k_level_change(self, event):
        if self.k_level_var.get() == 0:
            self.k0_frame.pack(fill='x', pady=10)
        else:
            self.k0_frame.pack_forget()
            
    def on_k0_strategy_change(self):
        if self.k0_strategy_var.get() == 'Patterned':
            self.pattern_frame.pack(fill='x', pady=5)
            self.percentage_frame.pack_forget()
        else:
            self.pattern_frame.pack_forget()
            self.percentage_frame.pack(fill='x', pady=5)
            
    def on_rat_choice_change(self, event):
        if self.controller.current_recommendation is not None:
            is_found = bool((self.rat_choice_var.get() - 1) == self.controller.current_recommendation)
            self.found_var.set(is_found)
            
    def on_start_button_clicked(self):
        try:
            k_level = self.k_level_var.get()
            num_spots = self.num_spots_var.get()
            k0_strat = 'patterned' if self.k0_strategy_var.get() == 'Patterned' else 'percentage'
            
            if k_level == 0:
                if k0_strat == 'patterned':
                    k0_config = [int(x.strip()) - 1 for x in self.pattern_var.get().split(',')]
                    if any(p >= num_spots or p < 0 for p in k0_config):
                        raise ValueError(f"Pattern values must be between 1 and {num_spots}.")
                else:
                    k0_config = [float(x.strip()) for x in self.percentage_var.get().split(',')]
                    if len(k0_config) != num_spots:
                        raise ValueError(f"Must provide {num_spots} percentages.")
                    if not np.isclose(sum(k0_config), 1.0):
                        raise ValueError("Percentages must sum to 1.0.")
            else:
                k0_config = None
                
            self.controller.initialize_game(k_level, num_spots, k0_strat, k0_config)
            self.run_next_recommendation()
            messagebox.showinfo("Success", "Experiment initialized successfully!")
            
        except Exception as e:
            messagebox.showerror("Input Error", str(e))
            
    def run_next_recommendation(self):
        recommendation = self.controller.recommend_spot()
        self.trial_num_label.config(text=f"Trial #{self.controller.trial_num}")
        self.recommendation_label.config(text=f"Robot Recommends Hiding In Spot: {recommendation + 1}")
        self.on_rat_choice_change(None)
        # Clear search sequence for new trial
        self.search_sequence_var.set("")
        self.update_display()
        
    def update_display(self):
        # Update trial log
        log_text = "Trial Log:\n"
        for entry in reversed(self.controller.trial_log):
            found_text = "✅ Found" if entry["was_found"] else "❌ Not Found"
            sequence_text = f" | Sequence: {entry.get('search_sequence', 'N/A')}" if entry.get('search_sequence') else ""
            log_text += (f"Trial {entry['trial_num']}: Robot hid in {entry['robot_hiding_spot']}, "
                        f"Rat first searched {entry['rat_first_search']}. "
                        f"({found_text} in {entry['time_to_find']:.1f}s){sequence_text}\n")
        self.trial_log_text.delete(1.0, tk.END)
        self.trial_log_text.insert(1.0, log_text)
        
        # Update model state
        model_state = self.controller.get_model_state_info()
        self.model_state_text.delete(1.0, tk.END)
        self.model_state_text.insert(1.0, model_state)
        
    def on_submit_button_clicked(self):
        try:
            rat_choice = self.rat_choice_var.get() - 1
            if not (0 <= rat_choice < self.controller.num_spots):
                raise ValueError(f"Rat's choice must be between 1 and {self.controller.num_spots}.")
            
            # Process search sequence
            search_sequence = self.search_sequence_var.get().strip()
            if search_sequence:
                # Validate search sequence format
                try:
                    self.validate_search_sequence(search_sequence)
                except ValueError as e:
                    raise ValueError(f"Invalid search sequence: {e}")
            
            self.controller.process_trial_result(rat_choice, bool(self.found_var.get()), self.time_var.get(), search_sequence)
            self.run_next_recommendation()
        except Exception as e:
            messagebox.showerror("Input Error", str(e))
            
    def on_generate_log_clicked(self):
        if not self.controller.trial_log:
            messagebox.showwarning("Warning", "No trial data to export!")
            return
            
        # Create timestamp in military time format: HHMMSS_YYYYMMDD
        now = datetime.now()
        date_str = now.strftime('%Y%m%d')
        time_str = now.strftime('%H%M%S')
        datetime_str = f"{time_str}_{date_str}"
        
        # Create folder name
        folder_name = f"hide_and_seek_trial_outcomes_{date_str}"
        
        # Create the folder if it doesn't exist
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        
        # Create filename
        filename = f"triallog_{datetime_str}.csv"
        file_path = os.path.join(folder_name, filename)
        
        # Write the CSV file
        all_headers = set()
        for entry in self.controller.trial_log:
            all_headers.update(entry.keys())
        header_order = [
            'trial_num', 'robot_k_level', 'num_hiding_spots', 'robot_hiding_spot',
            'rat_first_search', 'was_found', 'time_to_find', 'search_sequence'
        ]
        # Add k-level beliefs and predictions to the header
        k_belief_headers = sorted([h for h in all_headers if h.startswith('belief_rat_is_k')])
        pred_headers = sorted([h for h in all_headers if h.startswith('pred_rat_searches_spot')])

        sorted_headers = header_order + k_belief_headers + pred_headers
        # Add any remaining headers that might have been missed
        sorted_headers.extend(sorted([h for h in all_headers if h not in sorted_headers]))

        with open(file_path, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=sorted_headers, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(self.controller.trial_log)
        
        messagebox.showinfo("Success", f"Trial log saved to:\n{os.path.abspath(file_path)}")

    # --- Robot Control Event Handlers ---
    def on_connect_robot(self):
        if self.robot_link is None or not self.robot_link.connected:
            try:
                robot_ip = self.robot_ip_var.get()
                self.robot_link = RosLink(host=robot_ip, port=10090)
                
                # Set up status callbacks
                def on_line_follow_status(status):
                    self.update_robot_status(f"Line Follow: {status}")
                
                def on_rat_detection_status(found):
                    self.update_robot_status(f"Rat Detection: {'Found' if found else 'Not Found'}")
                
                def on_progress_status(progress):
                    self.update_robot_status(f"Progress: {progress}")
                
                # Connect with callbacks
                if self.robot_link.connect(on_lf=on_line_follow_status, 
                                         on_rat=on_rat_detection_status, 
                                         on_prog=on_progress_status):
                    self.robot_status = "Connected"
                    self.robot_status_label.config(text="Status: Connected", foreground='green')
                    self.connect_button.config(text="Disconnect")
                    self.update_robot_status("Successfully connected to robot")
                else:
                    self.robot_status = "Connection Failed"
                    self.robot_status_label.config(text="Status: Connection Failed", foreground='red')
                    self.update_robot_status("Failed to connect to robot")
                    
            except Exception as e:
                self.robot_status = "Connection Error"
                self.robot_status_label.config(text="Status: Connection Error", foreground='red')
                self.update_robot_status(f"Connection error: {str(e)}")
        else:
            # Disconnect
            self.robot_link.close()
            self.robot_link = None
            self.robot_status = "Disconnected"
            self.robot_status_label.config(text="Status: Disconnected", foreground='red')
            self.connect_button.config(text="Connect to Robot")
            self.update_robot_status("Disconnected from robot")

    def on_send_target(self):
        if self.robot_link and self.robot_link.connected:
            target = self.target_spot_var.get()
            # Convert A,B,C,D to 0,1,2,3
            target_map = {"A": 0, "B": 1, "C": 2, "D": 3}
            target_idx = target_map.get(target, 0)
            self.robot_link.send_target(target_idx)
            self.update_robot_status(f"Sent target spot: {target} (index: {target_idx})")
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")

    def on_send_drive_mode(self):
        if self.robot_link and self.robot_link.connected:
            mode = self.drive_mode_var.get()
            self.robot_link.send_toggle(f"drive_mode={mode}")
            self.update_robot_status(f"Set drive mode: {mode}")
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")

    def on_send_rat_mode(self):
        if self.robot_link and self.robot_link.connected:
            mode = self.rat_mode_var.get()
            self.robot_link.send_toggle(f"rat_mode={mode}")
            self.update_robot_status(f"Set rat detection mode: {mode}")
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")

    def on_manual_found(self):
        if self.robot_link and self.robot_link.connected:
            # Send manual found signal
            self.robot_link.send_toggle("manual_found=true")
            self.update_robot_status("Sent manual 'rat found' signal")
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")

    def on_send_velocity(self):
        if self.robot_link and self.robot_link.connected:
            linear = self.linear_speed_var.get()
            angular = self.angular_speed_var.get()
            self.robot_link.send_cmdvel(linear, angular)
            self.update_robot_status(f"Sent velocity: linear={linear}, angular={angular}")
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")

    def on_stop_robot(self):
        if self.robot_link and self.robot_link.connected:
            self.robot_link.send_cmdvel(0.0, 0.0)
            self.update_robot_status("Sent STOP command")
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")

    def validate_search_sequence(self, sequence):
        """Validate search sequence format (A-D or 1-4, comma-separated)"""
        if not sequence:
            return True
            
        # Split by comma and clean up
        spots = [spot.strip().upper() for spot in sequence.split(',')]
        
        # Check for valid spot labels
        valid_labels = set(['A', 'B', 'C', 'D', '1', '2', '3', '4'])
        for spot in spots:
            if spot not in valid_labels:
                raise ValueError(f"Invalid spot label: {spot}. Use A-D or 1-4.")
        
        # Check for duplicates
        if len(spots) != len(set(spots)):
            raise ValueError("Duplicate spots in sequence.")
        
        return True

    def update_robot_status(self, message):
        timestamp = datetime.now().strftime('%H:%M:%S')
        status_line = f"[{timestamp}] {message}\n"
        self.robot_status_text.insert(tk.END, status_line)
        self.robot_status_text.see(tk.END)
        # Keep only last 100 lines
        lines = self.robot_status_text.get(1.0, tk.END).split('\n')
        if len(lines) > 100:
            self.robot_status_text.delete(1.0, tk.END)
            self.robot_status_text.insert(1.0, '\n'.join(lines[-100:]))

def main():
    root = tk.Tk()
    app = KToMExperimenterGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()