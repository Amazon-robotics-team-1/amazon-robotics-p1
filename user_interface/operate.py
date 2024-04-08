# Webpage for the user interface.

from flask import Flask, render_template, redirect, request, url_for
from run_script import run_script


RUN_PICK_PLACE_SCRIPT = "/home/senior-project/Desktop/amazon-robotics-p1/run_pick_place.sh"
CLOSE_PROGRAM_SCRIPT = "/home/senior-project/Desktop/amazon-robotics-p1/close_program.sh"
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/script', methods=['POST'])
def script(result=None):
    # Execute script to open terminal windows
    user_input = request.form['user-input']
    result = run_script(script_path=RUN_PICK_PLACE_SCRIPT, user_input=user_input)
    return render_template('results.html', result=result)

@app.route('/close-route', methods=['POST'])
def close():
    # Execute script to close terminal windows
    run_script(script_path=CLOSE_PROGRAM_SCRIPT)
    return redirect(url_for("index"))

if __name__ == '__main__':
    app.run(debug=True)
