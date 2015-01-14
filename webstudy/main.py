from flask import Flask
from flask import redirect
from flask import url_for

app = Flask(__name__)

@app.route('/')
def home():
    return redirect('/txg/webstudy.html')
