import math

class AcfToSdfConverter:
    def __init__(self, acf_content):
        self.data = self._parse_acf(acf_content)
        
        # Costanti di conversione
        self.LB_TO_KG = 0.453592
        self.FT_TO_M = 0.3048
        
        # Valori di Default (Placeholder per dati mancanti)
        #self.DEFAULT_MASS_KG = 12.5  # Valore realistico per un drone medio
        self.DEFAULT_INERTIA = 0.1   # Inerzia sferica generica
        self.default_power_watt = 400.0 
        self.default_rotor_inertia = 0.0001 #Per evitare spinning infiniti su Gazebo

        self.mass_lb = self.get_value("acf/_m_empty", 0.0) 
        if self.mass_lb == 0:
             # Fallback vecchio metodo se la nuova chiave non c'è
             self.mass_lb = self.get_value("_cgpt/0/_w_now", 0.0)

    def _parse_acf(self, content):
        """Parsing semplice chiave-valore del contenuto .acf"""
        data = {}
        for line in content.strip().split('\n'):
            line = line.strip()
            if not line or not line.startswith('P '):
                continue
            
            parts = line.split()
            # Esempio: P _cgpt/0/_w_now 0.0
            if len(parts) >= 3:
                key = parts[1]  # _cgpt/0/_w_now
                values = parts[2:]
                
                # Tenta di convertire in float, altrimenti lascia stringa
                parsed_values = []
                for v in values:
                    try:
                        parsed_values.append(float(v))
                    except ValueError:
                        parsed_values.append(v)
                
                # Se è un valore singolo, estrailo dalla lista
                data[key] = parsed_values[0] if len(parsed_values) == 1 else parsed_values
        return data

    def convert_coords(self, xp_x_lat, xp_y_vert, xp_z_long):
        """
        Converte coordinate X-Plane (Feet) in Gazebo (Metri).
        X-Plane: X=Lat(+Dx), Y=Vert(+Su), Z=Long(+Coda/Sud)
        Gazebo:  X=Avanti, Y=Sinistra, Z=Su
        """
        gz_x = -(xp_z_long * self.FT_TO_M)
        gz_y = -(xp_x_lat * self.FT_TO_M)
        gz_z = (xp_y_vert * self.FT_TO_M)
        return gz_x, gz_y, gz_z

    def get_value(self, key_suffix, default=0.0):
        """Cerca una chiave che termina con il suffisso dato"""
        if key_suffix in self.data:
            return self.data[key_suffix]
        
        for key, val in self.data.items():
            if key.endswith(key_suffix):
                return val
        return default

    def generate_inertial_xml(self):
        """Genera il blocco <inertial> per l'SDF"""
        
        mass_kg = 1.0 # Fallback estremo
        
        if self.mass_lb > 0:
            mass_kg = self.mass_lb * self.LB_TO_KG
            print(f"[INFO] Massa Totale trovata: {self.mass_lb} lb -> {mass_kg:.3f} kg")
        else:
            print("[WARNING] Massa non trovata. Uso 1.0 kg fittizio.")

            # Inerzia del corpo principale (approssimazione scatola)
            # I = mass * cost. Usiamo 1/12 * mass * (width^2 + depth^2) approssimato
            ixx = iyy = izz = mass_kg * 0.05 

            return f"""
        <inertial>
        <mass>{mass_kg:.4f}</mass>
        <inertia>
            <ixx>{ixx:.4f}</ixx> <ixy>0</ixy> <ixz>0</ixz>
            <iyy>{iyy:.4f}</iyy> <iyz>0</iyz>
            <izz>{izz:.4f}</izz>
        </inertia>
        </inertial>
    """

    def _get_prop_data(self, index):
        """Estrae dati specifici dell'elica per l'indice dato"""
        # Cerca dati specifici per l'indice i, altrimenti fallback su indice 0 (template)
        # Elica 0 è spesso usata come default per tutte le altre
        
        # Numero pale
        num_blades = self.get_value(f"_prop/{index}/_num_blades", 
                                    self.get_value("_prop/0/_num_blades", 2.0))
        
        # Direzione (1.0 = Normal/CW, -1.0 = Reverse/CCW)
        direction_val = self.get_value(f"_prop/{index}/_prop_dir", 
                                       self.get_value("_prop/0/_prop_dir", 1.0))
        
        direction_str = "cw" if direction_val >= 0 else "ccw"
        
        return int(num_blades), direction_str

    def generate_engines_xml(self):
        """Genera i blocchi <link> e <joint> per i motori integrando
        dati _engn (Tipo) e _prop (Pale, Direzione)"""
        
        # 1. Conta i motori
        try:
            count_eng = int(self.get_value("_engn/count", 0))
            count_prop = int(self.get_value("_prop/count", 0))
            count = max(count_eng, count_prop)
        except:
            count = 0
            
        if count == 0:
            return ""

        xml_output = f"\n    \n"
        
        
        for i in range(count):
            # Cerca il tipo se specificato per ogni motore, altrimenti usa quello dello 0
            eng_type = self.get_value(f"_engn/{i}/_type", self.get_value("_engn/0/_type", "UNK"))

            #Dati elica
            blades, direction = self._get_prop_data(i)
            
            # COORDINATE MANCANTI: Uso 0,0,0 temporaneo
            # Nota: In futuro queste verranno sovrascritte leggendo la sezione _prop o i vettori globali
            pos_x, pos_y, pos_z = (0.0, 0.0, 0.0) 
            
            # Generazione Link Rotore
            xml_output += f"""
    <link name="rotor_{i}">
      <pose>{pos_x} {pos_y} {pos_z} 0 0 0</pose>
      
      <visual name="rotor_{i}_visual">
        <geometry>
          <cylinder><radius>{0.1 * blades}</radius><length>0.02</length></cylinder>
        </geometry>
        <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
      </visual>

      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>{self.default_rotor_inertia}</ixx> <ixy>0</ixy> <ixz>0</ixz>
          <iyy>{self.default_rotor_inertia}</iyy> <iyz>0</iyz>
          <izz>{self.default_rotor_inertia}</izz>
        </inertia>
      </inertial>
    </link>

    <joint name="rotor_{i}_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_{i}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
      </axis>
    </joint>
"""
        #NOTA: Le mesh andranno aggiunte dopo, i valori hard-coded sono per verificare che intanto la conversione sia giusta e visibile su gazebo.
        return xml_output

    def generate_wings_xml(self):
        """
        PASSO 4 (AGGIORNATO): Generazione superfici alari basata su count esplicito.        """
        # Lettura conteggio esplicito (default a 0 se non trovato)
        # Nota: Cerca sia la chiave standard che quella specifica del tuo file se diversa
        count = int(self.get_value("_wing/count", 0))
        if count == 0:
            return ""

        xml_output = f"\n    \n"

        for i in range(count):
            prefix = f"_wing/{i}"
            
            # 1. Dimensioni
            semi_len_ft = self.get_value(f"{prefix}/_semilen_SEG", 0.0)
            chord_root_ft = self.get_value(f"{prefix}/_Croot", 0.0)
            chord_tip_ft = self.get_value(f"{prefix}/_Ctip", 0.0)
            
            # Filtro anti-spam: se lunghezza e corda sono 0, salta l'elemento vuoto
            if semi_len_ft == 0 and chord_root_ft == 0:
                continue

            # Conversione in metri
            semi_len_m = semi_len_ft * self.FT_TO_M
            chord_mean_m = ((chord_root_ft + chord_tip_ft) / 2) * self.FT_TO_M
            
            # Geometria Box
            width = semi_len_m
            depth = chord_mean_m if chord_mean_m > 0 else 0.05
            height = 0.005 # Sottile
            
            # 2. Posizione (Ancora 0,0,0 perché mancano le variabili globali di posizione)
            pos_x, pos_y, pos_z = (0.0, 0.0, 0.0)

            # Logica colori: Piccoli = Rosso (Pale/Struttura), Grandi = Grigio (Ali vere)
            color = "1 0 0 1" if semi_len_m < 0.5 else "0.5 0.5 0.5 1"
            name_suffix = "prop_blade" if semi_len_m < 0.5 else "wing_surface"

            xml_output += f"""
    <link name="wing_{i}_{name_suffix}">
      <pose>{pos_x} {pos_y} {pos_z} 0 0 0</pose>
      <visual name="vis_wing_{i}">
        <geometry><box><size>{depth} {width} {height}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
      </link>"""
        return xml_output

    def generate_gear_xml(self):
        """Genera i carrelli (Landing Gear)"""
        count = int(self.get_value("_gear/count", 0))
        if count == 0: return ""

        xml_output = f"\n    \n"
        for i in range(count):
            prefix = f"_gear/{i}"
            # Se il tipo è 0, spesso è uno slot vuoto
            g_type = self.get_value(f"{prefix}/_gear_type", 0)
            if int(g_type) == 0:
                continue
                
            # Coordinate X-Plane (X=Lat, Y=Vert, Z=Long)
            xp_x = self.get_value(f"{prefix}/_gear_x", 0.0)
            xp_y = self.get_value(f"{prefix}/_gear_y", 0.0)
            xp_z = self.get_value(f"{prefix}/_gear_z", 0.0)
            
            # Raggio Ruota
            radius_ft = self.get_value(f"{prefix}/_tire_radius", 0.1)
            radius_m = radius_ft * self.FT_TO_M
            
            # Conversione Coordinate
            gz_x, gz_y, gz_z = self.convert_coords(xp_x, xp_y, xp_z)
            
            xml_output += f"""
    <link name="gear_{i}">
      <pose>{gz_x:.3f} {gz_y:.3f} {gz_z:.3f} 0 0 0</pose>
      <visual name="vis_gear_{i}">
        <geometry><sphere><radius>{radius_m:.3f}</radius></sphere></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <collision name="col_gear_{i}">
        <geometry><sphere><radius>{radius_m:.3f}</radius></sphere></geometry>
      </collision>
    </link>
    <joint name="gear_{i}_joint" type="fixed">
      <parent>fuselage</parent>
      <child>gear_{i}</child>
    </joint>"""
        return xml_output

    def generate_fuselage_collisions(self):
        """
        Genera collisioni basate sulla nuvola di punti _part.
        Ogni 'part' diventa un box di collisione.
        """
        parts_bounds = {}
        count_points = 0
        
        # Scansione dati
        for key, val in self.data.items():
            # Filtro rigoroso: deve contenere sia _part/ che _geo_xyz
            if "_geo_xyz" in key and "_part/" in key:
                try:
                    # PULIZIA CHIAVE E SEGMENTAZIONE
                    # Esempio Key: "P _part/17/_geo_xyz/19,9,0"
                    segments = key.split('/')
                    
                    # 1. Trova l'indice della parte
                    if "_part" in segments:
                        idx_tag = segments.index("_part")
                        # Verifica che ci sia un numero dopo _part
                        if len(segments) > idx_tag + 1 and segments[idx_tag+1].isdigit():
                            part_idx = int(segments[idx_tag + 1])
                        else:
                            continue
                    else:
                        continue

                    # 2. Analizza coordinate griglia (es. "19,9,0")
                    # L'ultimo segmento è la terna "i,j,asse"
                    grid_segment = segments[-1]
                    grid_coords = grid_segment.split(',')
                    
                    # Deve essere una terna esatta
                    if len(grid_coords) != 3:
                        continue
                    
                    # L'ultimo numero è l'asse (0=Long, 1=Lat, 2=Vert)
                    axis_str = grid_coords[2]
                    if not axis_str.isdigit():
                        continue
                    axis = int(axis_str)
                    
                    # 3. Assicura che il valore sia float
                    # Se val è una lista (caso raro), prendi il primo elemento
                    if isinstance(val, list):
                        val_float = float(val[0])
                    else:
                        val_float = float(val)

                    # 4. Archiviazione
                    if part_idx not in parts_bounds:
                        parts_bounds[part_idx] = {0: [], 1: [], 2: []}
                    
                    parts_bounds[part_idx][axis].append(val_float)
                    count_points += 1

                except Exception as e:
                    # Stampa l'errore se serve debugging, altrimenti prosegui
                    print(f"DEBUG ERROR su chiave {key}: {e}")
                    continue

        if not parts_bounds:
            print(f"[WARNING] Nessun punto _geo_xyz trovato in {len(self.data)} chiavi.")
            return ""
        
        print(f"[INFO] Trovati {count_points} punti strutturali in {len(parts_bounds)} parti.")

        xml_output = f"\n      \n"
        
        for idx, bounds in parts_bounds.items():
            # Verifica che abbiamo dati per tutti e 3 gli assi per creare un volume 3D
            if not bounds[0] or not bounds[1] or not bounds[2]:
                continue
                
            # Calcolo Min/Max in PIEDI (Coordinate Locali)
            # Axis 0 = X-Plane Z (Longitudinale)
            # Axis 1 = X-Plane X (Laterale)
            # Axis 2 = X-Plane Y (Verticale)
            min_l, max_l = min(bounds[0]), max(bounds[0]) # Long
            min_w, max_w = min(bounds[1]), max(bounds[1]) # Lat
            min_h, max_h = min(bounds[2]), max(bounds[2]) # Vert
            
            # Dimensioni
            size_l = max_l - min_l
            size_w = max_w - min_w
            size_h = max_h - min_h
            
            # FILTRO: Rimuove parti piatte o punti singoli (spazzatura)
            # Nota: La fusoliera è lunga, ma le sezioni trasversali possono essere sottili.
            # Rilassiamo il filtro: basta che abbia un volume minimo
            if size_l < 0.01 and size_w < 0.01: 
                continue 

            # Centro Locale
            center_l = (min_l + max_l) / 2
            center_w = (min_w + max_w) / 2
            center_h = (min_h + max_h) / 2
            
            # Recupera Offset Globale della Parte
            # Se manca, assume 0.0
            off_x = float(self.get_value(f"_part/{idx}/_part_x", 0.0)) # Lat
            off_y = float(self.get_value(f"_part/{idx}/_part_y", 0.0)) # Vert
            off_z = float(self.get_value(f"_part/{idx}/_part_z", 0.0)) # Long
            
            # Somma Posizione Globale (Offset + Centro Bounding Box)
            tot_long = off_z + center_l
            tot_lat  = off_x + center_w
            tot_vert = off_y + center_h
            
            # CONVERSIONE X-PLANE (Feet) -> GAZEBO (Metri)
            # Gazebo X = -Longitudinale
            # Gazebo Y = -Laterale
            # Gazebo Z = +Verticale
            gz_x = -(tot_long * self.FT_TO_M)
            gz_y = -(tot_lat  * self.FT_TO_M)
            gz_z = (tot_vert  * self.FT_TO_M)
            
            # Dimensioni in Metri
            dim_x = max(size_l * self.FT_TO_M, 0.05) # Minimo 5cm per evitare errori fisica
            dim_y = max(size_w * self.FT_TO_M, 0.05)
            dim_z = max(size_h * self.FT_TO_M, 0.05)
            
            xml_output += f"""
      <collision name="col_part_{idx}">
        <pose>{gz_x:.3f} {gz_y:.3f} {gz_z:.3f} 0 0 0</pose>
        <geometry><box><size>{dim_x:.3f} {dim_y:.3f} {dim_z:.3f}</size></box></geometry>
      </collision>
      <visual name="vis_part_{idx}_debug">
        <pose>{gz_x:.3f} {gz_y:.3f} {gz_z:.3f} 0 0 0</pose>
        <geometry><box><size>{dim_x:.3f} {dim_y:.3f} {dim_z:.3f}</size></box></geometry>
        <material><ambient>0.8 0.8 0.8 0.3</ambient><diffuse>0.8 0.8 0.8 0.3</diffuse></material>
      </visual>"""
        
        return xml_output

# --- BLOCCO DI TEST ---
if __name__ == "__main__":
    input_filename = "DRONE VERSIONE 2.txt"
    output_filename = "output.txt"
    
    print(f"Leggendo {input_filename}...")
    
    try:
        with open(input_filename, "r", encoding="utf-8", errors="ignore") as f:
            file_content = f.read()
            
        converter = AcfToSdfConverter(file_content)
        
        # Scrittura su file output.txt
        with open(output_filename, "w", encoding="utf-8") as out:
            # 1. Intestazione SDF
            out.write("<?xml version='1.0' ?>\n")
            out.write("<sdf version='1.9'>\n")
            out.write("  <model name='zefiro_vtol_converted'>\n")
            out.write("    <pose>0 0 0.5 0 0 0</pose>\n")

            # 2. Generazione Componenti

            out.write(converter.generate_inertial_xml() or "")
            out.write(converter.generate_fuselage_collisions() or "")
            #out.write(converter.generate_engines_xml() or "")
            #out.write(converter.generate_wings_xml() or "")
            #out.write(converter.generate_gear_xml() or "")

            # 3. Chiusura SDF
            out.write("\n  </model>\n</sdf>")
            
        print(f"✅ Conversione completata! Risultati salvati in: {output_filename}")
        
    except FileNotFoundError:
        print(f"❌ Errore: Il file '{input_filename}' non è stato trovato.")
    except Exception as e:
        print(f"❌ Errore imprevisto: {e}")