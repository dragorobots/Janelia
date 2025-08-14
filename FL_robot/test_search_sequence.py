#!/usr/bin/env python3
"""
Test script for search sequence validation functionality
"""

import tkinter as tk
from ktom_experimenter import KToMExperimenterGUI

def test_search_sequence_validation():
    """Test the search sequence validation method"""
    print("🧪 Testing Search Sequence Validation")
    print("=" * 40)
    
    # Create GUI (don't show it)
    root = tk.Tk()
    root.withdraw()
    app = KToMExperimenterGUI(root)
    
    # Test valid sequences
    valid_sequences = [
        "A,B,C,D",
        "1,2,3,4", 
        "A,C,B",
        "B,D,A,C",
        "1,3,2,4",
        "A,B,C",  # Partial sequence
        "D,C",    # Short sequence
        ""        # Empty sequence
    ]
    
    print("Testing valid sequences:")
    for seq in valid_sequences:
        try:
            app.validate_search_sequence(seq)
            print(f"✅ Valid: '{seq}'")
        except ValueError as e:
            print(f"❌ Invalid: '{seq}' - {e}")
    
    # Test invalid sequences
    invalid_sequences = [
        "A,B,C,A",  # Duplicate
        "A,B,C,E",  # Invalid label
        "A,B,C,5",  # Invalid number
        "A,B,C,F",  # Invalid label
        "A,B,C,",   # Trailing comma
        ",A,B,C",   # Leading comma
        "A,,B,C",   # Empty spot
        "A B C",    # Wrong separator
    ]
    
    print("\nTesting invalid sequences:")
    for seq in invalid_sequences:
        try:
            app.validate_search_sequence(seq)
            print(f"❌ Should be invalid: '{seq}'")
        except ValueError as e:
            print(f"✅ Correctly rejected: '{seq}' - {e}")
    
    root.destroy()
    print("\n✅ Search sequence validation test completed!")

if __name__ == "__main__":
    test_search_sequence_validation()
