from flask import Flask
from flask import render_template
import data_acq
app = Flask(__name__, template_folder='./template')


@app.route('/')
def index():
    data = data_acq.extract_data()
    data["Cell voltages"] = " ".join(data["Cell voltages"])
    return render_template('index.html', battery_data = data )


if __name__ == '__main__':
    app.run(debug=True)