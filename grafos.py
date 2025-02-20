import heapq  # Importação necessária para o algoritmo de Dijkstra

class Grafo:
    def __init__(self, numero_total_vertices):
        """
        Inicializa um grafo não direcionado com um número fixo de vértices.
        
        Parâmetros:
            numero_total_vertices (int): Quantidade total de vértices no grafo.
        """
        # Lista de adjacências: cada vértice tem uma lista de tuplas (vizinho, peso)
        self.lista_adjacencias = []
        for _ in range(numero_total_vertices + 1):  # Índices de 1 até numero_total_vertices
            self.lista_adjacencias.append([])
        self.quantidade_vertices = numero_total_vertices  # Total de vértices
        self.quantidade_arestas = 0  # Total de arestas

    def adicionar_aresta(self, vertice_origem, vertice_destino, peso_aresta):
        """
        Adiciona uma aresta não direcionada ao grafo (bidirecional).

        Parâmetros:
            vertice_origem (int): Vértice de origem da aresta.
            vertice_destino (int): Vértice de destino da aresta.
            peso_aresta (int): Peso associado à aresta.
        """
        # Adiciona a aresta de origem para destino
        self.lista_adjacencias[vertice_origem].append((vertice_destino, peso_aresta))
        # Adiciona a aresta de destino para origem (bidirecional)
        self.lista_adjacencias[vertice_destino].append((vertice_origem, peso_aresta))
        self.quantidade_arestas = self.quantidade_arestas + 1

    def obter_numero_vertices(self):
        """
        Retorna o número total de vértices do grafo.
        
        Retorna:
            int: Quantidade de vértices.
        """
        return self.quantidade_vertices

    def obter_numero_arestas(self):
        """
        Retorna o número total de arestas do grafo.
        
        Retorna:
            int: Quantidade de arestas.
        """
        return self.quantidade_arestas

    def obter_vizinhanca(self, vertice):
        """
        Retorna a vizinhança do vértice (vértices adjacentes).

        Parâmetros:
            vertice (int): Vértice cuja vizinhança será retornada.
        
        Retorna:
            list: Lista de vértices vizinhos (sem duplicatas).
        """
        vizinhos = []
        for vizinho, _ in self.lista_adjacencias[vertice]:
            vizinhos.append(vizinho)
        
        vizinhos_sem_duplicatas = []
        for v in vizinhos:
            if v not in vizinhos_sem_duplicatas:
                vizinhos_sem_duplicatas.append(v)
        
        return vizinhos_sem_duplicatas

    def obter_grau_total(self, vertice):
        """
        Retorna o grau total do vértice (número de arestas incidentes).

        Parâmetros:
            vertice (int): Vértice cujo grau será calculado.
        
        Retorna:
            int: Grau total do vértice.
        """
        return len(self.lista_adjacencias[vertice])

    def obter_peso_aresta(self, vertice_origem, vertice_destino):
        """
        Retorna o peso da aresta entre dois vértices.

        Parâmetros:
            vertice_origem (int): Vértice de origem da aresta.
            vertice_destino (int): Vértice de destino da aresta.
        
        Retorna:
            int ou None: Peso da aresta ou None se não existir.
        """
        for vizinho, peso in self.lista_adjacencias[vertice_origem]:
            if vizinho == vertice_destino:
                return peso
        return None

    def obter_menor_grau(self):
        """
        Retorna o menor grau total entre todos os vértices.

        Retorna:
            int: Menor grau total no grafo.
        """
        menor_grau_encontrado = None
        for vertice in range(1, self.quantidade_vertices + 1):
            grau_total = self.obter_grau_total(vertice)
            if menor_grau_encontrado is None or grau_total < menor_grau_encontrado:
                menor_grau_encontrado = grau_total
        return menor_grau_encontrado

    def obter_maior_grau(self):
        """
        Retorna o maior grau total entre todos os vértices.

        Retorna:
            int: Maior grau total no grafo.
        """
        maior_grau_encontrado = None
        for vertice in range(1, self.quantidade_vertices + 1):
            grau_total = self.obter_grau_total(vertice)
            if maior_grau_encontrado is None or grau_total > maior_grau_encontrado:
                maior_grau_encontrado = grau_total
        return maior_grau_encontrado

    def executar_busca_largura(self, vertice_inicial):
        """
        Executa a busca em largura (BFS) a partir de um vértice inicial.

        Parâmetros:
            vertice_inicial (int): Vértice de início da busca.
        
        Retorna:
            tuple: (distancias, predecessores) onde distancias são as distâncias mínimas
                   e predecessores são os vértices antecessores no caminho.
        """
        distancias = []
        predecessores = []
        for _ in range(self.quantidade_vertices + 1):
            distancias.append(-1)
            predecessores.append(None)
        
        distancias[vertice_inicial] = 0
        fila_vertices = [vertice_inicial]
        
        while len(fila_vertices) > 0:
            vertice_atual = fila_vertices.pop(0)
            for vizinho, _ in self.lista_adjacencias[vertice_atual]:
                if distancias[vizinho] == -1:
                    distancias[vizinho] = distancias[vertice_atual] + 1
                    predecessores[vizinho] = vertice_atual
                    fila_vertices.append(vizinho)
        
        return distancias, predecessores

    def executar_busca_profundidade(self, vertice_inicial):
        """
        Executa a busca em profundidade (DFS) a partir de um vértice inicial.

        Parâmetros:
            vertice_inicial (int): Vértice de início da busca.
        
        Retorna:
            tuple: (predecessores, tempos_inicio, tempos_fim) com informações da busca.
        """
        predecessores = []
        tempos_inicio = []
        tempos_fim = []
        for _ in range(self.quantidade_vertices + 1):
            predecessores.append(None)
            tempos_inicio.append(-1)
            tempos_fim.append(-1)
        
        tempo_atual = [0]
        
        def visitar_vertice(vertice):
            tempo_atual[0] = tempo_atual[0] + 1
            tempos_inicio[vertice] = tempo_atual[0]
            for vizinho, _ in self.lista_adjacencias[vertice]:
                if tempos_inicio[vizinho] == -1:
                    predecessores[vizinho] = vertice
                    visitar_vertice(vizinho)
            tempo_atual[0] = tempo_atual[0] + 1
            tempos_fim[vertice] = tempo_atual[0]
        
        visitar_vertice(vertice_inicial)
        return predecessores, tempos_inicio, tempos_fim

    def executar_bellman_ford(self, vertice_inicial):
        """
        Executa o algoritmo de Bellman-Ford para encontrar caminhos mínimos.

        Parâmetros:
            vertice_inicial (int): Vértice de origem dos caminhos.
        
        Retorna:
            tuple: (distancias, predecessores) com as distâncias mínimas e predecessores.
        """
        distancias = []
        predecessores = []
        for _ in range(self.quantidade_vertices + 1):
            distancias.append(float('inf'))
            predecessores.append(None)
        
        distancias[vertice_inicial] = 0
        
        for _ in range(self.quantidade_vertices - 1):
            for vertice_atual in range(1, self.quantidade_vertices + 1):
                for vizinho, peso in self.lista_adjacencias[vertice_atual]:
                    nova_distancia = distancias[vertice_atual] + peso
                    if nova_distancia < distancias[vizinho]:
                        distancias[vizinho] = nova_distancia
                        predecessores[vizinho] = vertice_atual
        
        return distancias, predecessores

    def executar_dijkstra(self, vertice_inicial):
        """
        Executa o algoritmo de Dijkstra para encontrar caminhos mínimos.

        Parâmetros:
            vertice_inicial (int): Vértice de origem dos caminhos.
        
        Retorna:
            tuple: (distancias, predecessores) com as distâncias mínimas e predecessores.
        """
        distancias = []
        predecessores = []
        for _ in range(self.quantidade_vertices + 1):
            distancias.append(float('inf'))
            predecessores.append(None)
        
        distancias[vertice_inicial] = 0
        fila_prioridade = [(0, vertice_inicial)]  # Lista de tuplas (distância, vértice)
        vertices_visitados = set()
        
        while len(fila_prioridade) > 0:
            distancia_atual, vertice_atual = heapq.heappop(fila_prioridade)
            if vertice_atual in vertices_visitados:
                continue
            vertices_visitados.add(vertice_atual)
            for vizinho, peso in self.lista_adjacencias[vertice_atual]:
                nova_distancia = distancia_atual + peso
                if nova_distancia < distancias[vizinho]:
                    distancias[vizinho] = nova_distancia
                    predecessores[vizinho] = vertice_atual
                    heapq.heappush(fila_prioridade, (nova_distancia, vizinho))
        
        return distancias, predecessores

class Digrafo:
    def __init__(self, numero_total_vertices):
        """
        Inicializa um digrafo com um número fixo de vértices.
        
        Parâmetros:
            numero_total_vertices (int): Quantidade total de vértices no digrafo.
        """
        # Lista de adjacências: cada vértice tem uma lista de tuplas (vizinho, peso)
        self.lista_adjacencias = []
        for _ in range(numero_total_vertices + 1):  # Índices de 1 até numero_total_vertices
            self.lista_adjacencias.append([])
        self.quantidade_vertices = numero_total_vertices  # Total de vértices
        self.quantidade_arestas = 0  # Total de arestas
        self.graus_entrada_cache = None  # Cache para armazenar graus de entrada

    def adicionar_aresta(self, vertice_origem, vertice_destino, peso_aresta):
        """
        Adiciona uma aresta direcionada ao digrafo.

        Parâmetros:
            vertice_origem (int): Vértice de origem da aresta.
            vertice_destino (int): Vértice de destino da aresta.
            peso_aresta (int): Peso associado à aresta.
        """
        self.lista_adjacencias[vertice_origem].append((vertice_destino, peso_aresta))
        self.quantidade_arestas = self.quantidade_arestas + 1
        self.graus_entrada_cache = None  # Resetar cache ao adicionar uma nova aresta

    def obter_numero_vertices(self):
        """
        Retorna o número total de vértices do digrafo.
        
        Retorna:
            int: Quantidade de vértices.
        """
        return self.quantidade_vertices

    def obter_numero_arestas(self):
        """
        Retorna o número total de arestas do digrafo.
        
        Retorna:
            int: Quantidade de arestas.
        """
        return self.quantidade_arestas

    def obter_vizinhanca(self, vertice):
        """
        Retorna a vizinhança do vértice (vértices de entrada e saída).

        Parâmetros:
            vertice (int): Vértice cuja vizinhança será retornada.
        
        Retorna:
            list: Lista de vértices vizinhos (sem duplicatas).
        """
        vizinhos_saida = []
        for vizinho, _ in self.lista_adjacencias[vertice]:
            vizinhos_saida.append(vizinho)
        
        vizinhos_entrada = []
        for vertice_atual in range(1, self.quantidade_vertices + 1):
            for vizinho, _ in self.lista_adjacencias[vertice_atual]:
                if vizinho == vertice:
                    vizinhos_entrada.append(vertice_atual)
        
        vizinhanca_total = vizinhos_saida + vizinhos_entrada
        vizinhanca_sem_duplicatas = []
        for v in vizinhanca_total:
            if v not in vizinhanca_sem_duplicatas:
                vizinhanca_sem_duplicatas.append(v)
        
        return vizinhanca_sem_duplicatas

    def obter_grau_total(self, vertice):
        """
        Retorna o grau total do vértice (soma de entradas e saídas).

        Parâmetros:
            vertice (int): Vértice cujo grau será calculado.
        
        Retorna:
            int: Grau total do vértice.
        """
        grau_entrada = self.obter_grau_entrada(vertice)
        grau_saida = self.obter_grau_saida(vertice)
        return grau_entrada + grau_saida

    def obter_grau_entrada(self, vertice):
        """
        Retorna o número de arestas entrando no vértice.

        Parâmetros:
            vertice (int): Vértice cujo grau de entrada será calculado.
        
        Retorna:
            int: Grau de entrada do vértice.
        """
        graus_entrada = self.calcular_graus_entrada()
        return graus_entrada[vertice]

    def obter_grau_saida(self, vertice):
        """
        Retorna o número de arestas saindo do vértice.

        Parâmetros:
            vertice (int): Vértice cujo grau de saída será calculado.
        
        Retorna:
            int: Grau de saída do vértice.
        """
        return len(self.lista_adjacencias[vertice])

    def calcular_graus_entrada(self):
        """
        Calcula os graus de entrada de todos os vértices e armazena em cache.

        Retorna:
            list: Lista com os graus de entrada de cada vértice.
        """
        if self.graus_entrada_cache is None:
            self.graus_entrada_cache = []
            for _ in range(self.quantidade_vertices + 1):
                self.graus_entrada_cache.append(0)
            for vertice_atual in range(1, self.quantidade_vertices + 1):
                for vizinho, _ in self.lista_adjacencias[vertice_atual]:
                    self.graus_entrada_cache[vizinho] = self.graus_entrada_cache[vizinho] + 1
        return self.graus_entrada_cache

    def obter_peso_aresta(self, vertice_origem, vertice_destino):
        """
        Retorna o peso da aresta entre dois vértices.

        Parâmetros:
            vertice_origem (int): Vértice de origem da aresta.
            vertice_destino (int): Vértice de destino da aresta.
        
        Retorna:
            int ou None: Peso da aresta ou None se não existir.
        """
        for vizinho, peso in self.lista_adjacencias[vertice_origem]:
            if vizinho == vertice_destino:
                return peso
        return None

    def obter_menor_grau(self):
        """
        Retorna o menor grau total entre todos os vértices.

        Retorna:
            int: Menor grau total no digrafo.
        """
        menor_grau_encontrado = None
        graus_entrada = self.calcular_graus_entrada()
        for vertice in range(1, self.quantidade_vertices + 1):
            grau_total = graus_entrada[vertice] + self.obter_grau_saida(vertice)
            if menor_grau_encontrado is None or grau_total < menor_grau_encontrado:
                menor_grau_encontrado = grau_total
        return menor_grau_encontrado

    def obter_maior_grau(self):
        """
        Retorna o maior grau total entre todos os vértices.

        Retorna:
            int: Maior grau total no digrafo.
        """
        maior_grau_encontrado = None
        graus_entrada = self.calcular_graus_entrada()
        for vertice in range(1, self.quantidade_vertices + 1):
            grau_total = graus_entrada[vertice] + self.obter_grau_saida(vertice)
            if maior_grau_encontrado is None or grau_total > maior_grau_encontrado:
                maior_grau_encontrado = grau_total
        return maior_grau_encontrado

    def executar_busca_largura(self, vertice_inicial):
        """
        Executa a busca em largura (BFS) a partir de um vértice inicial.

        Parâmetros:
            vertice_inicial (int): Vértice de início da busca.
        
        Retorna:
            tuple: (distancias, predecessores) onde distancias são as distâncias mínimas
                   e predecessores são os vértices antecessores no caminho.
        """
        distancias = []
        predecessores = []
        for _ in range(self.quantidade_vertices + 1):
            distancias.append(-1)
            predecessores.append(None)
        
        distancias[vertice_inicial] = 0
        fila_vertices = [vertice_inicial]
        
        while len(fila_vertices) > 0:
            vertice_atual = fila_vertices.pop(0)
            for vizinho, _ in self.lista_adjacencias[vertice_atual]:
                if distancias[vizinho] == -1:
                    distancias[vizinho] = distancias[vertice_atual] + 1
                    predecessores[vizinho] = vertice_atual
                    fila_vertices.append(vizinho)
        
        return distancias, predecessores

    def executar_busca_profundidade(self, vertice_inicial):
        """
        Executa a busca em profundidade (DFS) a partir de um vértice inicial.

        Parâmetros:
            vertice_inicial (int): Vértice de início da busca.
        
        Retorna:
            tuple: (predecessores, tempos_inicio, tempos_fim) com informações da busca.
        """
        predecessores = []
        tempos_inicio = []
        tempos_fim = []
        for _ in range(self.quantidade_vertices + 1):
            predecessores.append(None)
            tempos_inicio.append(-1)
            tempos_fim.append(-1)
        
        tempo_atual = [0]
        
        def visitar_vertice(vertice):
            tempo_atual[0] = tempo_atual[0] + 1
            tempos_inicio[vertice] = tempo_atual[0]
            for vizinho, _ in self.lista_adjacencias[vertice]:
                if tempos_inicio[vizinho] == -1:
                    predecessores[vizinho] = vertice
                    visitar_vertice(vizinho)
            tempo_atual[0] = tempo_atual[0] + 1
            tempos_fim[vertice] = tempo_atual[0]
        
        visitar_vertice(vertice_inicial)
        return predecessores, tempos_inicio, tempos_fim

    def executar_bellman_ford(self, vertice_inicial):
        """
        Executa o algoritmo de Bellman-Ford para encontrar caminhos mínimos.

        Parâmetros:
            vertice_inicial (int): Vértice de origem dos caminhos.
        
        Retorna:
            tuple: (distancias, predecessores) com as distâncias mínimas e predecessores.
        """
        distancias = []
        predecessores = []
        for _ in range(self.quantidade_vertices + 1):
            distancias.append(float('inf'))
            predecessores.append(None)
        
        distancias[vertice_inicial] = 0
        
        for _ in range(self.quantidade_vertices - 1):
            for vertice_atual in range(1, self.quantidade_vertices + 1):
                for vizinho, peso in self.lista_adjacencias[vertice_atual]:
                    nova_distancia = distancias[vertice_atual] + peso
                    if nova_distancia < distancias[vizinho]:
                        distancias[vizinho] = nova_distancia
                        predecessores[vizinho] = vertice_atual
        
        return distancias, predecessores

    def executar_dijkstra(self, vertice_inicial):
        """
        Executa o algoritmo de Dijkstra para encontrar caminhos mínimos.

        Parâmetros:
            vertice_inicial (int): Vértice de origem dos caminhos.
        
        Retorna:
            tuple: (distancias, predecessores) com as distâncias mínimas e predecessores.
        """
        distancias = []
        predecessores = []
        for _ in range(self.quantidade_vertices + 1):
            distancias.append(float('inf'))
            predecessores.append(None)
        
        distancias[vertice_inicial] = 0
        fila_prioridade = [(0, vertice_inicial)]  # Lista de tuplas (distância, vértice)
        vertices_visitados = set()
        
        while len(fila_prioridade) > 0:
            distancia_atual, vertice_atual = heapq.heappop(fila_prioridade)
            if vertice_atual in vertices_visitados:
                continue
            vertices_visitados.add(vertice_atual)
            for vizinho, peso in self.lista_adjacencias[vertice_atual]:
                nova_distancia = distancia_atual + peso
                if nova_distancia < distancias[vizinho]:
                    distancias[vizinho] = nova_distancia
                    predecessores[vizinho] = vertice_atual
                    heapq.heappush(fila_prioridade, (nova_distancia, vizinho))
        
        return distancias, predecessores

def carregar_digrafo():
    """
    Carrega o digrafo a partir do arquivo USA-road-d.NY.gr na mesma pasta.

    Retorna:
        Digrafo: Instância do digrafo preenchida com os dados do arquivo.
    """
    arquivo = open('USA-road-d.NY.gr', 'r')
    linhas = arquivo.readlines()
    arquivo.close()
    
    # Ignorar as 7 primeiras linhas do cabeçalho
    dados = []
    for linha in linhas[7:]:
        partes = linha.strip().split()
        dados.append(partes)
    
    numero_vertices = 264346  # Número fixo de vértices conforme especificado
    digrafo = Digrafo(numero_vertices)
    
    for linha in dados:
        if linha[0] == 'a':
            vertice_origem = int(linha[1])
            vertice_destino = int(linha[2])
            peso = int(linha[3])
            digrafo.adicionar_aresta(vertice_origem, vertice_destino, peso)
    
    return digrafo

def encontrar_caminho(predecessores, vertice_inicio, vertice_fim):
    """
    Reconstrói o caminho de um vértice inicial até um vértice final.

    Parâmetros:
        predecessores (list): Lista de predecessores de cada vértice.
        vertice_inicio (int): Vértice de origem do caminho.
        vertice_fim (int): Vértice de destino do caminho.
    
    Retorna:
        list: Lista de vértices no caminho, ou vazia se não houver caminho.
    """
    if predecessores[vertice_fim] is None:
        return []
    
    caminho = [vertice_fim]
    vertice_atual = vertice_fim
    while vertice_atual != vertice_inicio:
        vertice_atual = predecessores[vertice_atual]
        if vertice_atual is None:
            return []
        caminho.append(vertice_atual)
    
    caminho.reverse()
    return caminho

def encontrar_ciclo(digrafo, limite_maximo_vertices=1000):
    """
    Encontra um ciclo com pelo menos 5 arestas em uma sub-região do digrafo.

    Parâmetros:
        digrafo (Digrafo): Instância do digrafo.
        limite_maximo_vertices (int): Número máximo de vértices a explorar.
    
    Retorna:
        list: Lista de vértices formando o ciclo, ou None se não encontrado.
    """
    vertices_visitados = set()
    pilha_vertices = []
    predecessores = {}

    def buscar_ciclo(vertice):
        vertices_visitados.add(vertice)
        pilha_vertices.append(vertice)
        for vizinho, _ in digrafo.lista_adjacencias[vertice]:
            if vizinho not in vertices_visitados:
                predecessores[vizinho] = vertice
                ciclo_encontrado = buscar_ciclo(vizinho)
                if ciclo_encontrado is not None:
                    return ciclo_encontrado
            elif vizinho in pilha_vertices and vizinho != predecessores.get(vertice):
                indice_inicio_ciclo = pilha_vertices.index(vizinho)
                ciclo = pilha_vertices[indice_inicio_ciclo:]
                if len(ciclo) >= 5:
                    return ciclo
        pilha_vertices.pop()
        return None

    limite = min(digrafo.obter_numero_vertices() + 1, limite_maximo_vertices + 1)
    for vertice in range(1, limite):
        if vertice not in vertices_visitados:
            ciclo = buscar_ciclo(vertice)
            if ciclo is not None:
                return ciclo
    return None

def executar_casos_teste():
    """
    Executa os casos de teste solicitados no trabalho e exibe os resultados.
    """
    print("Iniciando o carregamento do digrafo a partir do arquivo USA-road-d.NY.gr...")
    digrafo = carregar_digrafo()
    print("Digrafo carregado com sucesso!")

    print("Calculando o menor grau do digrafo...")
    menor_grau = digrafo.obter_menor_grau()
    print(f"a) Menor grau do digrafo: {menor_grau}")

    print("Calculando o maior grau do digrafo...")
    maior_grau = digrafo.obter_maior_grau()
    print(f"b) Maior grau do digrafo: {maior_grau}")

    print("Executando busca em largura a partir do vértice 1...")
    distancias, predecessores = digrafo.executar_busca_largura(1)
    print("Busca em largura concluída!")
    caminho_longo = None
    for vertice in range(1, digrafo.obter_numero_vertices() + 1):
        if distancias[vertice] >= 10:
            caminho_longo = encontrar_caminho(predecessores, 1, vertice)
            break
    if caminho_longo is not None:
        print(f"c) Caminho com 10 ou mais arestas: {caminho_longo}")
    else:
        print("c) Caminho com 10 ou mais arestas: Não encontrado")

    print("Buscando um ciclo com 5 ou mais arestas...")
    ciclo = encontrar_ciclo(digrafo, limite_maximo_vertices=1000)
    if ciclo is not None:
        print(f"d) Ciclo com 5 ou mais arestas: {ciclo}")
    else:
        print("d) Ciclo com 5 ou mais arestas: Não encontrado")

    print("Executando o algoritmo de Dijkstra a partir do vértice 129...")
    distancias, predecessores = digrafo.executar_dijkstra(129)
    print("Algoritmo de Dijkstra concluído!")
    maior_distancia = 0
    vertice_mais_distante = 0
    for vertice in range(1, digrafo.obter_numero_vertices() + 1):
        if distancias[vertice] != float('inf') and distancias[vertice] > maior_distancia:
            maior_distancia = distancias[vertice]
            vertice_mais_distante = vertice
    print(f"e) Vértice mais distante do 129: {vertice_mais_distante}, Distância: {maior_distancia}")

# Executa o programa
if __name__ == "__main__":
    executar_casos_teste()