from sklearn import tree

colors_array = [
    (42, 46, 49),
    (42, 44, 36),
    (43, 48, 51),
    (43, 44, 36),
    # branco
    (13, 16, 3),
    (9, 11, 0),
    (13, 17, 4),
    (9, 11, 0),
    # preto
    (3, 19, 6),
    (2, 16, 3),
    (3, 19, 6),
    (2, 16, 3),
    # verde
    (36, 3, 0),
    (36, 3, 0),
    (37, 4, 1),
    (38, 3, 0),
    # vermelho
    (46, 30, 8),
    (43, 27, 4),
    (47, 31, 9),
    (44, 28, 5),
    # amarelo
    (7, 20, 33),
    (7, 19, 24),
    (7, 21, 35),
    (7, 19, 24),
    # azul
]

decision_tree = tree.DecisionTreeClassifier()
decision_tree = decision_tree.fit(
    colors_array,
    [0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5],
)


print(tree.export_text(decision_tree, feature_names=["R", "G", "B"]))
