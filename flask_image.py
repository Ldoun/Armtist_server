import flask
import werkzeug
from python_model_run import call
import os
import firebase_admin
from firebase_admin import credentials
from firebase_admin import storage



UPLOAD_FOLDER = 'C:/workspace/flask_server/mobile_real_img'
FAKE_FOLDER="C:/workspace/flask_server/gan_g"

def fileUpload(file):
    
    blob = bucket.blob( file)
    
    #new token and metadata 설정
   
    print("fileupload2 실행")
    #upload file
    blob.upload_from_filename(filename=file, content_type='image/png')
    print(blob.public_url)

#Initialize Firestore DB
cred = credentials.Certificate('C:/workspace/flask_server/key.json')
default_app = firebase_admin.initialize_app(cred, {
    'storageBucket': 'armtist-6e5be.appspot.com'
})

bucket = storage.bucket()

app = flask.Flask(__name__)
app.debug = True
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['FAKE_FOLDER']=FAKE_FOLDER   

@app.route('/', methods=['GET', 'POST'])
def handle_request():
    imagefile = flask.request.files['image']
    filename = werkzeug.utils.secure_filename(imagefile.filename)
    print("\nReceived image File name : " + imagefile.filename)
    imagefile.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))

    try:
        call()
        print('worked')
        #storage().put("C:/workspace/flask_server/gan_g/android_real_fake.png")
        fileUpload("C:/workspace/flask_server/gan_g/android_real_fake.png")
        try:
            return send_from_directory(app.config['FAKE_FOLDER'], filename="android_real_fake.png", as_attachment=True)
        except FileNotFoundError:
            abort(404)
       
    except:
        return "error while creating  gan image"
 
app.run(host="0.0.0.0", port=5000, debug=True)