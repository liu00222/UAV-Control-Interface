import pickle

# load pickled model
with open("model.pkl", "rb") as f:
    clf = pickle.load(f)

# make prediction with pure-predict object
y_pred = clf.predict([[0.25, 2.0, 8.3, 1.0]])
print(y_pred)