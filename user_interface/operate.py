# Webpage for the user interface.

from flask import Flask, render_template
from run_script import run_script


SCRIPT_PATH = "/home/senior-project/Desktop/amazon-robotics-p1/run_pick_place.sh"
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/script', methods=['POST'])
def script(result=None):
    # Execute your Python script here
    result = run_script(SCRIPT_PATH)
    return render_template('results.html', result=result)


if __name__ == '__main__':
    app.run(debug=True)
