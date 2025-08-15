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
        self.line_color = roslibpy.Topic(self.ros,'/hide_and_seek/line_color','std_msgs/msg/String')
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
            self.line_color.advertise()
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

    def send_line_color(self, hue):
        if self.connected:
            self.line_color.publish(roslibpy.Message({'data': f'hue={hue}'}))

    def add_status_callback(self, callback):
        self.status_callbacks.append(callback)

    def close(self):
        self.connected = False
        for t in [self.target, self.toggle, self.cmdvel, self.line_color, self.lf, self.rat, self.prog]:
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

    def get_current_recommendation(self):
        """Get the current k-ToM recommendation"""
        return self.current_recommendation

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
        
        # Bind keyboard shortcuts
        self.root.bind('<Escape>', lambda e: self.on_abort_mission())
        self.root.bind('<Control-a>', lambda e: self.on_abort_mission())
        
        # Setup tab
        setup_frame = ttk.Frame(notebook)
        notebook.add(setup_frame, text="Setup")
        self.create_setup_tab(setup_frame)
        
        # Robot Control tab
        robot_frame = ttk.Frame(notebook)
        notebook.add(robot_frame, text="Robot Control")
        self.create_robot_control_tab(robot_frame)
        
        # Trial tab
        trial_frame = ttk.Frame(notebook)
        notebook.add(trial_frame, text="Log Trial Data")
        self.create_trial_tab(trial_frame)
        
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
        
        # Line Color Configuration frame
        line_color_frame = ttk.LabelFrame(setup_frame, text="Line Color Configuration", padding=10)
        line_color_frame.pack(fill='x', pady=10)
        
        # Pre-defined colors toggle
        self.use_predefined_colors_var = tk.BooleanVar(value=False)
        self.predefined_colors_check = ttk.Checkbutton(line_color_frame, 
                                                      text="Use Pre-defined Colors", 
                                                      variable=self.use_predefined_colors_var,
                                                      command=self.on_predefined_colors_toggle)
        self.predefined_colors_check.pack(anchor='w', pady=5)
        
        # Line color inputs
        self.line_colors_frame = ttk.Frame(line_color_frame)
        self.line_colors_frame.pack(fill='x', pady=5)
        
        # Start line color
        ttk.Label(self.line_colors_frame, text="Start Line (RGB, comma-separated):").pack(anchor='w')
        self.start_color_var = tk.StringVar(value="255,0,0")
        self.start_color_entry = ttk.Entry(self.line_colors_frame, textvariable=self.start_color_var)
        self.start_color_entry.pack(fill='x', pady=2)
        
        # Line A color
        ttk.Label(self.line_colors_frame, text="Line A (RGB, comma-separated):").pack(anchor='w')
        self.line_a_color_var = tk.StringVar(value="0,255,0")
        self.line_a_color_entry = ttk.Entry(self.line_colors_frame, textvariable=self.line_a_color_var)
        self.line_a_color_entry.pack(fill='x', pady=2)
        
        # Line B color
        ttk.Label(self.line_colors_frame, text="Line B (RGB, comma-separated):").pack(anchor='w')
        self.line_b_color_var = tk.StringVar(value="0,0,255")
        self.line_b_color_entry = ttk.Entry(self.line_colors_frame, textvariable=self.line_b_color_var)
        self.line_b_color_entry.pack(fill='x', pady=2)
        
        # Line C color
        ttk.Label(self.line_colors_frame, text="Line C (RGB, comma-separated):").pack(anchor='w')
        self.line_c_color_var = tk.StringVar(value="255,255,0")
        self.line_c_color_entry = ttk.Entry(self.line_colors_frame, textvariable=self.line_c_color_var)
        self.line_c_color_entry.pack(fill='x', pady=2)
        
        # Line D color
        ttk.Label(self.line_colors_frame, text="Line D (RGB, comma-separated):").pack(anchor='w')
        self.line_d_color_var = tk.StringVar(value="255,0,255")
        self.line_d_color_entry = ttk.Entry(self.line_colors_frame, textvariable=self.line_d_color_var)
        self.line_d_color_entry.pack(fill='x', pady=2)
        
        # Color measurer button
        self.color_measurer_button = ttk.Button(line_color_frame, text="🎨 Open Color Measurer", 
                                               command=self.open_color_measurer)
        self.color_measurer_button.pack(pady=5)
        
        # Start button
        self.start_button = ttk.Button(setup_frame, text="Start Challenge", 
                                      command=self.on_start_button_clicked, style='Accent.TButton')
        self.start_button.pack(pady=10)
        
        # Initialize k0 frame visibility
        self.on_k_level_change(None)
        self.on_k0_strategy_change()
        
        # Initialize line color entries as disabled
        self.on_predefined_colors_toggle()
        
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
        title_label = ttk.Label(parent, text="2. Robot Control", font=('Arial', 16, 'bold'))
        title_label.pack(pady=10)
        
        # Create main container with left and right panels
        main_container = ttk.Frame(parent)
        main_container.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Left panel for controls
        left_panel = ttk.Frame(main_container)
        left_panel.pack(side='left', fill='both', expand=True, padx=(0, 10))
        
        # Right panel for instructions and progress
        right_panel = ttk.Frame(main_container)
        right_panel.pack(side='right', fill='y', padx=(10, 0))
        
        # === LEFT PANEL: CONTROLS ===
        
        # Connection frame with inline layout
        connection_frame = ttk.LabelFrame(left_panel, text="1. Connect to Robot", padding=10)
        connection_frame.pack(fill='x', pady=5)
        
        # Create inline frame for robot status bar
        status_bar_frame = ttk.Frame(connection_frame)
        status_bar_frame.pack(fill='x', pady=5)
        
        # Robot IP input (inline)
        ttk.Label(status_bar_frame, text="IP:").pack(side='left', padx=(0, 5))
        self.robot_ip_var = tk.StringVar(value="10.0.0.234")
        self.robot_ip_entry = ttk.Entry(status_bar_frame, textvariable=self.robot_ip_var, width=12)
        self.robot_ip_entry.pack(side='left', padx=(0, 10))
        
        # Connection button (inline)
        self.connect_button = ttk.Button(status_bar_frame, text="Connect", 
                                        command=self.on_connect_robot)
        self.connect_button.pack(side='left', padx=(0, 10))
        
        # Check robot status button (inline)
        self.check_robot_status_button = ttk.Button(status_bar_frame, text="Check Status", 
                                                   command=self.on_check_robot_status)
        self.check_robot_status_button.pack(side='left', padx=(0, 10))
        
        # Status display (inline)
        self.robot_status_label = ttk.Label(status_bar_frame, text="Disconnected", 
                                           foreground='red')
        self.robot_status_label.pack(side='left', padx=(0, 10))
        
        # Abort status indicator (inline)
        self.abort_status_label = ttk.Label(status_bar_frame, text="ABORT: INACTIVE", 
                                           foreground='green', font=('Arial', 9, 'bold'))
        self.abort_status_label.pack(side='left', padx=(0, 10))
        
        # Abort mission button (inline, top right)
        self.abort_button = ttk.Button(status_bar_frame, text="🚨 ABORT MISSION", 
                                      command=self.on_abort_mission)
        self.abort_button.pack(side='right', padx=(10, 0))
        
        # Style the abort button to be red with white text
        try:
            style = ttk.Style()
            style.configure('Emergency.TButton', 
                          background='red', 
                          foreground='white',
                          font=('Arial', 10, 'bold'))
            self.abort_button.configure(style='Emergency.TButton')
        except:
            # Fallback if styling fails
            pass
        
        # Target spot frame with inline layout
        target_frame = ttk.LabelFrame(left_panel, text="3. Send Target Hiding Spot", padding=10)
        target_frame.pack(fill='x', pady=5)
        
        # Create inline frame for target spot controls
        target_controls_frame = ttk.Frame(target_frame)
        target_controls_frame.pack(fill='x', pady=5)
        
        # Target spot selection (inline)
        ttk.Label(target_controls_frame, text="Target:").pack(side='left', padx=(0, 5))
        self.target_spot_var = tk.StringVar(value="A")
        target_combo = ttk.Combobox(target_controls_frame, textvariable=self.target_spot_var, 
                                   values=["A", "B", "C", "D"], state='readonly', width=8)
        target_combo.pack(side='left', padx=(0, 10))
        
        # Manual override checkbox (inline, next to dropdown)
        self.manual_override_var = tk.BooleanVar(value=False)
        self.manual_override_check = ttk.Checkbutton(target_controls_frame, 
                                                    text="Manual override", 
                                                    variable=self.manual_override_var,
                                                    command=self.on_manual_override_toggle)
        self.manual_override_check.pack(side='left', padx=(0, 10))
        
        # Auto-send target button (inline, right of manual toggle)
        self.auto_send_target_button = ttk.Button(target_controls_frame, text="Auto-Send k-ToM", 
                                                  command=self.on_auto_send_target, style='Accent.TButton')
        self.auto_send_target_button.pack(side='left', padx=(0, 10))
        
        # Send target button (inline)
        self.send_target_button = ttk.Button(target_controls_frame, text="Send Target", 
                                            command=self.on_send_target)
        self.send_target_button.pack(side='left')
        
        # Mode selection frame with 3 columns
        mode_frame = ttk.LabelFrame(left_panel, text="Mode Selections", padding=10)
        mode_frame.pack(fill='x', pady=5)
        
        # Create 3 columns
        col1 = ttk.Frame(mode_frame)
        col1.pack(side='left', fill='both', expand=True, padx=(0, 5))
        
        col2 = ttk.Frame(mode_frame)
        col2.pack(side='left', fill='both', expand=True, padx=5)
        
        col3 = ttk.Frame(mode_frame)
        col3.pack(side='left', fill='both', expand=True, padx=(5, 0))
        
        # Column 1: Line following mode
        ttk.Label(col1, text="4. Line Following Mode:", font=('Arial', 10, 'bold')).pack(anchor='w')
        self.color_mode_var = tk.StringVar(value="auto")
        color_combo = ttk.Combobox(col1, textvariable=self.color_mode_var, 
                                  values=["auto", "manual"], state='readonly')
        color_combo.pack(fill='x', pady=2)
        
        self.send_color_mode_button = ttk.Button(col1, text="Set", 
                                               command=self.on_send_color_mode)
        self.send_color_mode_button.pack(pady=2)
        
        # Column 2: Hiding spot mode
        ttk.Label(col2, text="5. Hiding Spot Mode:", font=('Arial', 10, 'bold')).pack(anchor='w')
        self.hiding_spot_mode_var = tk.StringVar(value="auto")
        hiding_spot_combo = ttk.Combobox(col2, textvariable=self.hiding_spot_mode_var, 
                                        values=["auto", "manual"], state='readonly')
        hiding_spot_combo.pack(fill='x', pady=2)
        
        self.send_hiding_spot_mode_button = ttk.Button(col2, text="Set", 
                                                     command=self.on_send_hiding_spot_mode)
        self.send_hiding_spot_mode_button.pack(pady=2)
        
        # Column 3: Rat detection mode
        ttk.Label(col3, text="6. Rat Detection Mode:", font=('Arial', 10, 'bold')).pack(anchor='w')
        self.rat_mode_var = tk.StringVar(value="auto")
        rat_combo = ttk.Combobox(col3, textvariable=self.rat_mode_var, 
                                values=["auto", "manual"], state='readonly')
        rat_combo.pack(fill='x', pady=2)
        
        self.send_rat_mode_button = ttk.Button(col3, text="Set", 
                                              command=self.on_send_rat_mode)
        self.send_rat_mode_button.pack(pady=2)
        
        # Manual found button
        self.manual_found_button = ttk.Button(left_panel, text="Manual: Rat Found!", 
                                             command=self.on_manual_found, style='Accent.TButton')
        self.manual_found_button.pack(pady=10)
        
        # Robot status frame
        status_frame = ttk.LabelFrame(left_panel, text="Robot Status", padding=10)
        status_frame.pack(fill='both', expand=True, pady=5)
        
        # Status text widget
        self.robot_status_text = tk.Text(status_frame, height=8, wrap='word')
        status_scrollbar = ttk.Scrollbar(status_frame, orient='vertical', command=self.robot_status_text.yview)
        self.robot_status_text.configure(yscrollcommand=status_scrollbar.set)
        
        self.robot_status_text.pack(side='left', fill='both', expand=True)
        status_scrollbar.pack(side='right', fill='y')
        
        # Start Trial button (in line with robot status window)
        start_trial_frame = ttk.Frame(left_panel)
        start_trial_frame.pack(fill='x', pady=10)
        
        # Big green start trial button
        self.start_trial_button = ttk.Button(start_trial_frame, text="🚀 START TRIAL", 
                                           command=self.on_start_trial, style='StartTrial.TButton')
        self.start_trial_button.pack(fill='x', pady=5)
        
        # Style the start trial button to be big and green
        try:
            style = ttk.Style()
            style.configure('StartTrial.TButton', 
                          background='green', 
                          foreground='white',
                          font=('Arial', 16, 'bold'))
        except:
            # Fallback if styling fails
            pass
        
        # === RIGHT PANEL: INSTRUCTIONS & PROGRESS ===
        
        # Create a horizontal container for instructions and progress
        instructions_progress_container = ttk.Frame(right_panel)
        instructions_progress_container.pack(fill='x', pady=5)
        
        # Trial Instructions frame (left side)
        instructions_frame = ttk.LabelFrame(instructions_progress_container, text="📋 Trial Instructions", padding=10)
        instructions_frame.pack(side='left', fill='both', expand=True, padx=(0, 5))
        
        # Instructions text
        instructions_text = """
Step-by-Step Process:

1) Connect to robot
   • Enter robot IP address
   • Click "Connect to Robot"
   • Click "Check Robot Status"

2) Verify robot is ready
   • If status shows issues, run these commands on robot:
     Tab 1: ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=10090
     Tab 2: ./start_hide_and_seek.sh

3) Send target hiding spot
   • Use k-ToM recommendation (Auto-Send button)
   • Or manually select with override checkbox

4) Configure modes (3 columns):
   • Line following: auto/manual ROI selection
   • Hiding spot: auto/manual selection  
   • Rat detection: auto LiDAR/manual button

5) Start trial when ready
   • Click the big green START TRIAL button below
        """
        
        instructions_label = ttk.Label(instructions_frame, text=instructions_text, 
                                      font=('Arial', 9), justify='left', wraplength=250)
        instructions_label.pack(anchor='w', pady=5)
        
        # Trial Progress frame (right side)
        progress_frame = ttk.LabelFrame(instructions_progress_container, text="🎯 Trial Progress", padding=10)
        progress_frame.pack(side='right', fill='y', padx=(5, 0))
        
        # Progress steps with checkboxes
        self.progress_steps = [
            "1. Leave entrance",
            "2. Reach intersection & start new line", 
            "3. Follow the line",
            "4. Wait at hiding spot (detect rat)",
            "5. Wait 10s, turn 180°",
            "6. Follow line back (same color)",
            "7. Reach intersection & return to start",
            "8. Wait for new command, turn 180°, reset"
        ]
        
        # Create progress indicators with checkboxes
        self.progress_vars = []
        self.progress_labels = []
        for i, step in enumerate(self.progress_steps):
            var = tk.BooleanVar(value=False)
            self.progress_vars.append(var)
            
            # Create frame for each step
            step_frame = ttk.Frame(progress_frame)
            step_frame.pack(fill='x', pady=1)
            
            # Checkbox
            checkbox = ttk.Checkbutton(step_frame, variable=var, state='disabled')
            checkbox.pack(side='left', padx=(0, 5))
            
            # Label
            label = ttk.Label(step_frame, text=step, 
                             font=('Arial', 9), foreground='gray')
            label.pack(side='left', fill='x', expand=True)
            self.progress_labels.append(label)
        
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
            
    def on_predefined_colors_toggle(self):
        """Handle predefined colors toggle"""
        if self.use_predefined_colors_var.get():
            # Enable color inputs
            for entry in [self.start_color_entry, self.line_a_color_entry, 
                         self.line_b_color_entry, self.line_c_color_entry, self.line_d_color_entry]:
                entry.config(state='normal')
        else:
            # Disable color inputs
            for entry in [self.start_color_entry, self.line_a_color_entry, 
                         self.line_b_color_entry, self.line_c_color_entry, self.line_d_color_entry]:
                entry.config(state='disabled')
                
    def open_color_measurer(self):
        """Open the color measurer tool"""
        try:
            import subprocess
            import sys
            
            # Get the robot IP from the robot control tab
            robot_ip = self.robot_ip_var.get()
            
            # Start color measurer in a separate process
            if sys.platform.startswith('win'):
                subprocess.Popen([sys.executable, 'color_measurer.py'], 
                               creationflags=subprocess.CREATE_NEW_CONSOLE)
            else:
                subprocess.Popen([sys.executable, 'color_measurer.py'])
                
            messagebox.showinfo("Color Measurer", 
                              f"Color measurer opened in a new window.\n"
                              f"Use it to measure line colors and copy RGB values to the setup.")
                              
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open color measurer:\n{str(e)}")
            
    def validate_rgb_color(self, color_str):
        """Validate RGB color format (R,G,B)"""
        try:
            parts = color_str.split(',')
            if len(parts) != 3:
                raise ValueError("Must have exactly 3 values (R,G,B)")
            
            r, g, b = map(int, parts)
            if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                raise ValueError("RGB values must be between 0 and 255")
                
            return True
        except ValueError as e:
            raise ValueError(f"Invalid RGB format: {e}")
            
    def get_line_colors(self):
        """Get the configured line colors"""
        if not self.use_predefined_colors_var.get():
            return None
            
        try:
            colors = {
                'start': self.start_color_var.get(),
                'A': self.line_a_color_var.get(),
                'B': self.line_b_color_var.get(),
                'C': self.line_c_color_var.get(),
                'D': self.line_d_color_var.get()
            }
            
            # Validate all colors
            for name, color_str in colors.items():
                self.validate_rgb_color(color_str)
                
            return colors
            
        except ValueError as e:
            messagebox.showerror("Color Error", f"Invalid color format: {e}")
            return None
            
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
                
            # Validate line colors if using predefined colors
            line_colors = None
            if self.use_predefined_colors_var.get():
                line_colors = self.get_line_colors()
                if line_colors is None:
                    return  # Error already shown by get_line_colors
                
            self.controller.initialize_game(k_level, num_spots, k0_strat, k0_config)
            
            # Store line colors in controller
            if line_colors:
                self.controller.line_colors = line_colors
                
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
        # Reset trial progress for new trial
        self.update_trial_progress(0)
        
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
                    # Update progress based on line following status
                    if "following" in status.lower():
                        self.update_trial_progress(3)  # Step 3: Follow the line
                    elif "intersection" in status.lower():
                        self.update_trial_progress(2)  # Step 2: Reach intersection
                
                def on_rat_detection_status(found):
                    self.update_robot_status(f"Rat Detection: {'Found' if found else 'Not Found'}")
                    if found:
                        self.update_trial_progress(4)  # Step 4: Wait at hiding spot (detect rat)
                
                def on_progress_status(progress):
                    self.update_robot_status(f"Progress: {progress}")
                    # Map progress messages to trial steps
                    progress_lower = progress.lower()
                    
                    # Only update if the progress actually changed to prevent flashing
                    current_step = getattr(self, '_last_progress_step', -1)
                    new_step = -1
                    
                    if "waiting_for_start" in progress_lower:
                        new_step = 0  # Waiting for start
                    elif "trial_started" in progress_lower:
                        new_step = 1  # Trial started - beginning line following
                    elif "leaving_entrance" in progress_lower or "entrance" in progress_lower:
                        new_step = 1  # Step 1: Leave entrance (line following)
                    elif "following_start_line" in progress_lower:
                        new_step = 1  # Step 1: Following start line to intersection
                    elif "at_intersection_centering" in progress_lower:
                        new_step = 2  # Step 2: At intersection, centering
                    elif "following_target_line" in progress_lower:
                        new_step = 3  # Step 3: Following target line to hiding spot
                    elif "following_line" in progress_lower:
                        new_step = 3  # Step 3: Follow the line (legacy)
                    elif "searching_for_line" in progress_lower:
                        new_step = 3  # Step 3: Still in line following phase
                    elif "intersection" in progress_lower:
                        new_step = 2  # Step 2: Reach intersection (legacy)
                    elif "waiting_for_rat" in progress_lower:
                        new_step = 4  # Step 4: Wait at hiding spot
                    elif "turning_180" in progress_lower:
                        new_step = 5  # Step 5: Wait 10s, turn 180°
                    elif "returning_home" in progress_lower:
                        new_step = 7  # Step 7: Return to start
                    elif "reset" in progress_lower:
                        new_step = 8  # Step 8: Wait for new command
                    
                    # Only update if step actually changed
                    if new_step != current_step:
                        self._last_progress_step = new_step
                        if new_step >= 0:
                            self.update_trial_progress(new_step)
                
                # Connect with callbacks
                if self.robot_link.connect(on_lf=on_line_follow_status, 
                                         on_rat=on_rat_detection_status, 
                                         on_prog=on_progress_status):
                    self.robot_status = "Connected"
                    self.robot_status_label.config(text="Status: Connected", foreground='green')
                    self.connect_button.config(text="Disconnect")
                    self.abort_status_label.config(text="ABORT: INACTIVE", foreground='green')
                    self.update_robot_status("Successfully connected to robot")
                    
                    # Reset progress tracking to prevent flashing
                    self._last_progress_step = -1
                    self.update_trial_progress(0)  # Reset to waiting state
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
            self.connect_button.config(text="Connect")
            self.update_robot_status("Disconnected from robot")

    def on_send_target(self):
        if self.robot_link and self.robot_link.connected:
            # Check if manual override is enabled
            if not self.manual_override_var.get():
                messagebox.showwarning("Warning", "Manual override is disabled. Use 'Auto-Send k-ToM Target' button to send the recommended spot.")
                return
                
            target = self.target_spot_var.get()
            # Convert A,B,C,D to 0,1,2,3
            target_map = {"A": 0, "B": 1, "C": 2, "D": 3}
            target_idx = target_map.get(target, 0)
            self.robot_link.send_target(target_idx)
            self.update_robot_status(f"Sent manual target spot: {target} (index: {target_idx})")
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")

    def on_start_trial(self):
        """Start the trial on the robot"""
        if self.robot_link and self.robot_link.connected:
            self.robot_link.send_toggle("start_trial=true")
            self.update_robot_status("🚀 Sent START TRIAL command to robot")
            self.update_trial_progress(1)  # Start with step 1
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")

    def on_send_color_mode(self):
        """Send color selection mode to robot"""
        if self.robot_link and self.robot_link.connected:
            mode = self.color_mode_var.get()
            self.robot_link.send_toggle(f"color_mode={mode}")
            self.update_robot_status(f"Set color selection mode: {mode}")
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
            
    def on_send_hiding_spot_mode(self):
        if self.robot_link and self.robot_link.connected:
            mode = self.hiding_spot_mode_var.get()
            self.robot_link.send_toggle(f"hiding_spot_mode={mode}")
            self.update_robot_status(f"Set hiding spot mode: {mode}")
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")
            
    def on_manual_override_toggle(self):
        """Handle manual override toggle for target spot selection"""
        if self.manual_override_var.get():
            # Enable manual target spot selection
            self.target_spot_var.set("A")  # Reset to default
            self.update_robot_status("Manual target spot selection enabled")
        else:
            # Disable manual selection, use k-ToM recommendation
            self.update_robot_status("Using k-ToM recommendation for target spot")

    def on_manual_found(self):
        if self.robot_link and self.robot_link.connected:
            # Send manual found signal
            self.robot_link.send_toggle("manual_found=true")
            self.update_robot_status("Sent manual 'rat found' signal")
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")


            
    def on_abort_mission(self):
        """Emergency abort - stop all robot operations and kill hide_and_seek process"""
        if self.robot_link and self.robot_link.connected:
            # Update abort status indicator
            self.abort_status_label.config(text="ABORT: ACTIVE", foreground='red')
            
            # Send emergency stop command
            self.robot_link.send_cmdvel(0.0, 0.0)
            self.update_robot_status("🚨 EMERGENCY STOP: Robot halted")
            
            # Send abort signal to robot
            self.robot_link.send_toggle("abort_mission=true")
            self.update_robot_status("🚨 ABORT MISSION: Signal sent to robot")
            
            # Show confirmation dialog
            result = messagebox.askyesno("ABORT MISSION", 
                "Emergency stop activated!\n\n"
                "Robot has been halted and abort signal sent.\n\n"
                "Do you want to attempt to kill the hide_and_seek process on the robot?\n"
                "(This will require SSH access to the robot)")
            
            if result:
                self.kill_robot_processes()
        else:
            messagebox.showwarning("Warning", "Not connected to robot!")
            
    def kill_robot_processes(self):
        """Attempt to kill hide_and_seek processes on the robot via SSH"""
        try:
            import subprocess
            import socket
            import threading
            
            # Run SSH command in a separate thread to prevent GUI blocking
            def run_ssh_kill():
                try:
                    # Get robot IP
                    robot_ip = self.robot_ip_var.get()
                    
                    # Test if we can reach the robot
                    try:
                        socket.create_connection((robot_ip, 22), timeout=5)
                    except:
                        self.root.after(0, lambda: messagebox.showerror("Connection Error", 
                            f"Cannot reach robot at {robot_ip} on SSH port 22.\n"
                            "Please manually kill the hide_and_seek processes on the robot."))
                        return
                    
                    # Kill hide_and_seek processes
                    ssh_command = f"ssh root@{robot_ip} 'pkill -f hide_and_seek'"
                    
                    result = subprocess.run(ssh_command, shell=True, capture_output=True, text=True, timeout=10)
                    
                    if result.returncode == 0:
                        self.root.after(0, lambda: self.update_robot_status("✅ hide_and_seek processes killed on robot"))
                        self.root.after(0, lambda: messagebox.showinfo("Success", "hide_and_seek processes have been killed on the robot."))
                    else:
                        self.root.after(0, lambda: self.update_robot_status(f"⚠️ Failed to kill processes: {result.stderr}"))
                        self.root.after(0, lambda: messagebox.showwarning("Warning", 
                            f"Failed to kill processes on robot.\n"
                            f"Error: {result.stderr}\n\n"
                            f"Please manually kill the processes on the robot using:\n"
                            f"ssh root@{robot_ip}\n"
                            f"pkill -f hide_and_seek"))
                        
                except subprocess.TimeoutExpired:
                    self.root.after(0, lambda: self.update_robot_status("⚠️ SSH command timed out"))
                    self.root.after(0, lambda: messagebox.showwarning("Timeout", 
                        "SSH command timed out. Please manually kill the processes on the robot."))
                except Exception as e:
                    self.root.after(0, lambda: self.update_robot_status(f"❌ Error killing processes: {str(e)}"))
                    self.root.after(0, lambda: messagebox.showerror("Error", 
                        f"Error attempting to kill robot processes:\n{str(e)}\n\n"
                        "Please manually kill the hide_and_seek processes on the robot."))
            
            # Start the SSH thread
            ssh_thread = threading.Thread(target=run_ssh_kill, daemon=True)
            ssh_thread.start()
                    
        except Exception as e:
            self.update_robot_status(f"❌ Error starting SSH thread: {str(e)}")
            messagebox.showerror("Error", 
                f"Error starting SSH thread:\n{str(e)}\n\n"
                "Please manually kill the hide_and_seek processes on the robot.")

    def on_check_robot_status(self):
        """Check if hide_and_seek.sh is running on robot and bridge is active"""
        try:
            import subprocess
            import socket
            import threading
            
            robot_ip = self.robot_ip_var.get()
            
            # Validate IP address format
            if not robot_ip or robot_ip.strip() == "":
                messagebox.showerror("Error", "Please enter a valid robot IP address first.")
                return
            
            # Check if we're already connected via ROS bridge
            bridge_connected = (self.robot_link and self.robot_link.connected)
            
            # Show initial status
            status_message = "🤖 Robot Status Check Results:\n\n"
            
            # Check 1: Can we reach the robot via SSH?
            ssh_reachable = False
            try:
                socket.create_connection((robot_ip, 22), timeout=5)
                ssh_reachable = True
                status_message += "✅ Robot is reachable via SSH\n"
            except (socket.timeout, socket.error, OSError):
                status_message += "❌ Cannot reach robot via SSH (port 22)\n"
                status_message += "   - Check if robot is powered on\n"
                status_message += "   - Check network connection\n"
                status_message += "   - Verify IP address is correct\n"
            
            # Check 2: Is ROS bridge active?
            if bridge_connected:
                status_message += "✅ ROS bridge is connected\n"
            else:
                status_message += "❌ ROS bridge is NOT connected\n"
                status_message += "   - Click 'Connect to Robot' first\n"
            
            # Check 3: Is hide_and_seek.sh running? (only if SSH is reachable)
            hide_and_seek_running = False
            if ssh_reachable:
                try:
                    # Run SSH command in a separate thread to prevent GUI blocking
                    def check_hide_and_seek():
                        nonlocal hide_and_seek_running
                        try:
                            ssh_command = f"ssh root@{robot_ip} 'pgrep -f hide_and_seek'"
                            result = subprocess.run(ssh_command, shell=True, capture_output=True, text=True, timeout=10)
                            hide_and_seek_running = result.returncode == 0 and result.stdout.strip()
                            
                            # Update status message based on result
                            if hide_and_seek_running:
                                status_message += "✅ hide_and_seek.sh is running on robot\n"
                            else:
                                status_message += "❌ hide_and_seek.sh is NOT running on robot\n"
                                status_message += "   - Start it with: ssh root@" + robot_ip + "\n"
                                status_message += "   - Then run: cd ~/yahboomcar_ws/src/Janelia/FL_robot\n"
                                status_message += "   - Then run: ./start_hide_and_seek.sh\n"
                            
                            # Show final status
                            if hide_and_seek_running and bridge_connected:
                                status_message += "\n🎉 Robot is ready for trials!"
                                self.root.after(0, lambda: messagebox.showinfo("Robot Status", status_message))
                            else:
                                status_message += "\n⚠️ Robot needs attention before trials."
                                self.root.after(0, lambda: messagebox.showwarning("Robot Status", status_message))
                                
                        except subprocess.TimeoutExpired:
                            status_message += "⚠️ SSH command timed out\n"
                            status_message += "\n⚠️ Robot needs attention before trials."
                            self.root.after(0, lambda: messagebox.showwarning("Robot Status", status_message))
                        except Exception as e:
                            status_message += f"⚠️ Error checking hide_and_seek: {str(e)}\n"
                            status_message += "\n⚠️ Robot needs attention before trials."
                            self.root.after(0, lambda: messagebox.showwarning("Robot Status", status_message))
                    
                    # Start the SSH check thread
                    ssh_thread = threading.Thread(target=check_hide_and_seek, daemon=True)
                    ssh_thread.start()
                    
                    # Show immediate status while SSH check is running
                    status_message += "⏳ Checking hide_and_seek.sh status...\n"
                    messagebox.showinfo("Robot Status", status_message)
                    
                except Exception as e:
                    status_message += f"⚠️ Error starting SSH check: {str(e)}\n"
                    status_message += "\n⚠️ Robot needs attention before trials."
                    messagebox.showwarning("Robot Status", status_message)
            else:
                # If SSH is not reachable, show status without hide_and_seek check
                status_message += "\n⚠️ Cannot check hide_and_seek.sh status (SSH unreachable)"
                messagebox.showwarning("Robot Status", status_message)
                
        except Exception as e:
            messagebox.showerror("Error", f"Error checking robot status: {str(e)}")

    def on_auto_send_target(self):
        """Automatically send the k-ToM recommended hiding spot to robot"""
        if not self.robot_link or not self.robot_link.connected:
            messagebox.showwarning("Warning", "Not connected to robot!")
            return
            
        try:
            # Get the current k-ToM recommendation
            if hasattr(self, 'controller') and self.controller:
                # Get the recommended hiding spot from k-ToM
                recommended_spot = self.controller.get_current_recommendation()
                
                if recommended_spot is not None:
                    # Convert to robot format (A, B, C, D)
                    spot_map = {0: "A", 1: "B", 2: "C", 3: "D"}
                    target_spot = spot_map.get(recommended_spot, "A")
                    
                    # Update the target spot variable
                    self.target_spot_var.set(target_spot)
                    
                    # Send target to robot
                    target_map = {"A": 0, "B": 1, "C": 2, "D": 3}
                    target_idx = target_map.get(target_spot, 0)
                    self.robot_link.send_target(target_idx)
                    
                    # Send line color to robot (convert RGB to HSV hue)
                    try:
                        line_colors = self.get_line_colors()
                        if line_colors and target_spot in line_colors:
                            rgb_str = line_colors[target_spot]
                            rgb_values = [int(x.strip()) for x in rgb_str.split(',')]
                            if len(rgb_values) == 3:
                                # Convert RGB to HSV
                                import cv2
                                import numpy as np
                                rgb_array = np.array([[rgb_values]], dtype=np.uint8)
                                hsv_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2HSV)
                                hue = int(hsv_array[0, 0, 0])
                                
                                # Send line color to robot
                                self.robot_link.send_line_color(hue)
                                self.update_robot_status(f"🎨 Sent line color for {target_spot}: RGB{rgb_values} → HSV hue {hue}")
                    except Exception as color_error:
                        self.update_robot_status(f"⚠️ Could not send line color: {str(color_error)}")
                    
                    self.update_robot_status(f"🤖 Auto-sent k-ToM recommendation: {target_spot} (index: {target_idx})")
                    self.update_trial_progress(0)  # Reset trial progress
                    
                else:
                    messagebox.showwarning("Warning", "No k-ToM recommendation available. Please run a trial first.")
            else:
                messagebox.showwarning("Warning", "k-ToM controller not initialized. Please start a trial first.")
                
        except Exception as e:
            messagebox.showerror("Error", f"Error auto-sending target: {str(e)}")

    def update_trial_progress(self, step_number):
        """Update the trial progress indicator"""
        if 0 <= step_number <= len(self.progress_steps):
            # Update all progress checkboxes and labels
            for i, (var, label) in enumerate(zip(self.progress_vars, self.progress_labels)):
                if step_number == 0:
                    # Reset state - all steps gray and unchecked
                    var.set(False)
                    label.config(foreground='gray')
                elif i < step_number:
                    # Completed steps
                    var.set(True)
                    label.config(foreground='green')
                elif i == step_number:
                    # Current step
                    var.set(False)
                    label.config(foreground='orange')
                else:
                    # Future steps
                    var.set(False)
                    label.config(foreground='gray')
            


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