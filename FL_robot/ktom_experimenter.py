import numpy as np
from scipy.special import softmax, logsumexp
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import csv
from datetime import datetime
import os

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

    def process_trial_result(self, rat_choice, was_found, time_taken):
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
            "time_to_find": time_taken
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
        self.root.geometry("800x900")
        
        self.controller = RobotController()
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
        
        # Rat's choice
        ttk.Label(input_frame, text="Rat's First Search (1-based):").pack(anchor='w')
        self.rat_choice_var = tk.IntVar(value=1)
        self.rat_choice_spin = ttk.Spinbox(input_frame, from_=1, to=10, textvariable=self.rat_choice_var)
        self.rat_choice_spin.pack(fill='x', pady=2)
        self.rat_choice_spin.bind('<KeyRelease>', self.on_rat_choice_change)
        
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
        self.update_display()
        
    def update_display(self):
        # Update trial log
        log_text = "Trial Log:\n"
        for entry in reversed(self.controller.trial_log):
            found_text = "✅ Found" if entry["was_found"] else "❌ Not Found"
            log_text += (f"Trial {entry['trial_num']}: Robot hid in {entry['robot_hiding_spot']}, "
                        f"Rat first searched {entry['rat_first_search']}. "
                        f"({found_text} in {entry['time_to_find']:.1f}s)\n")
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
            self.controller.process_trial_result(rat_choice, bool(self.found_var.get()), self.time_var.get())
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
            'rat_first_search', 'was_found', 'time_to_find'
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

def main():
    root = tk.Tk()
    app = KToMExperimenterGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()