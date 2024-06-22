import vedo
import os

def getLetterStl(letter)-> str:
    letter_file_name=None
    if letter.isdigit():
        letter_file_name="stl/numbers/"+letter+".stl"
        print("got number")
    elif letter.isalpha():
        if letter.isupper():
            letter_file_name="stl/uppercase/"+letter+".stl"
            print("got upper")
        elif letter.islower():
            letter_file_name="stl/lowercase/"+letter+".stl"
            print("got lower")
    elif letter=='&':
        letter_file_name="stl/specialchar/"+"Ampersand.stl"
    elif letter=='@':
        letter_file_name="stl/specialchar/"+"Ampersat.stl"
    elif letter==';':
        letter_file_name="stl/specialchar/"+"Semicolon.stl"
    elif letter==',':
        letter_file_name="stl/specialchar/"+"Comma.stl"
    elif letter=='.':
        letter_file_name="stl/specialchar/"+"Dot.stl"
    elif letter=='/':
        letter_file_name="stl/specialchar/"+"Forward slash.stl"
    elif letter=='-':
        letter_file_name="stl/specialchar/"+"Hyphen.stl"
    elif letter==':':
        letter_file_name="stl/specialchar/"+"Colon.stl"
    elif letter=='_':
        letter_file_name="stl/specialchar/"+"Underscore.stl"
    
    return letter_file_name

def getNumberStlForEmboss(letter)-> str:
    letter_file_name=None
    if letter.isdigit():
        letter_file_name="emboss_numbers/"+letter+".stl"
        print(letter_file_name)
        print("got number")
    
    return letter_file_name

def getStl(word: str,number, outputStl) -> str:
    try:
        meshes=[]
        index = 0
        for letter in word:
            if letter==' ':
                index+=1
            else:    
                letter_file_name=getLetterStl(letter)
                number_file_name=getLetterStl(number)
                if(letter_file_name != None):
                    letter_mesh=vedo.Mesh(letter_file_name)
                    letter_mesh.pos(index*6,0,0)
                    
                    meshes.append(letter_mesh)
                    index+=1
        for letter in number:
            if letter==' ':
                index+=1
            else:    
                number_file_name=getLetterStl(letter)
                if(number_file_name != None):
                    number_mesh=vedo.Mesh(number_file_name)
                    number_mesh.pos(index*3,-3.8,0)
                    
                    meshes.append(number_mesh)
                    index+=1

        #show(meshes, axes=True)
        mesh=vedo.merge(meshes)
        vedo.write(mesh, outputStl)
        return None
    except Exception as e:
        print("**************\nOops! Exception occurred.")
        print(e)
        print("**************")
        return None
    
def getNumberStlFile(number, outputStl) -> str:
    try:
        meshes=[]
        index = 0
        number_file_name=getNumberStlForEmboss(number)
        if(number_file_name != None):
            print("got number")
            number_mesh=vedo.Mesh(number_file_name)
            # number_mesh.pos(index*3,-5,0)
            print("mesh open")
            meshes.append(number_mesh)
            print("meshes appended")

        #show(meshes, axes=True)
        mesh=vedo.merge(meshes)
        print("mesh mereg")
        vedo.write(mesh, outputStl)
        print("file written ")
        return 
    except Exception as e:
        print("**************\nOops! Exception occurred.")
        print(e)
        print("**************")
        return None


def getStlWithTranslation(word: str,number, outputStl, originX, originY, originZ):
    try:
        meshes=[]
        index = 0
        for letter in word:
            if letter==' ':
                index+=1
            else:    
                letter_file_name=getLetterStl(letter)
                number_file_name=getLetterStl(number)
                if(letter_file_name != None):
                    letter_mesh=vedo.Mesh(letter_file_name)
                    letter_mesh.pos(index*6,0,0)
                    
                    meshes.append(letter_mesh)
                    index+=1
        for letter in number:
            if letter==' ':
                index+=1
            else:    
                number_file_name=getLetterStl(letter)
                if(number_file_name != None):
                    number_mesh=vedo.Mesh(number_file_name)
                    number_mesh.pos(index*3,-5,0)
                    
                    meshes.append(number_mesh)
                    index+=1

        mesh=vedo.merge(meshes)
        mesh.pos(originX,originY,originZ)
        vedo.show(mesh, axes=True)
        vedo.write(mesh, outputStl)
        return mesh
    except Exception as e:
        print("**************\nOops! Exception occurred.")
        print(e)
        print("**************")
        return None


def getStlWithRotationAndTranslation(word: str,number, outputStl,RotationTranslationStl, originX, originY, originZ, rotateX, rotateY, rotateZ):
    try:
        index = 0
        # for letter in word:
        #     if letter==' ':
        #         index+=1
        #     else:    
        #         letter_file_name=getLetterStl(letter)
        #         number_file_name=getLetterStl(number)
        #         if(letter_file_name != None):
        #             letter_mesh=vedo.Mesh(letter_file_name)
        #             letter_mesh.pos(index*6,0,0)
                    
        #             meshes.append(letter_mesh)
        #             index+=1
        # for letter in number:
        #     if letter==' ':
        #         index+=1
        #     else:    
        #         number_file_name=getLetterStl(letter)
        #         if(number_file_name != None):
        #             number_mesh=vedo.Mesh(number_file_name)
        #             number_mesh.pos(index*3,-5,0)
                    
        #             meshes.append(number_mesh)
        #             index+=1
        mesh=vedo.Mesh(outputStl)
        # mesh=vedo.merge(meshes)
        #scale
        mesh.scale(0.8)
        #Rotate
        # mesh.rotateX(rotateX)
        # mesh.rotateY(rotateY)
        mesh.rotateZ(rotateZ)
        #Translate
        mesh.pos(originX,originY,originZ)

        # vedo.show(mesh, axes=True)
        vedo.write(mesh, RotationTranslationStl)
    except Exception as e:
        print("**************\nOops! Exception occurred.")
        print(e)
        print("**************")
        return None

def getStlWithRotation(word: str,number, outputStl, rotateX, rotateY, rotateZ):
    try:
        meshes=[]
        index = 0
        for letter in word:
            if letter==' ':
                index+=1
            else:    
                letter_file_name=getLetterStl(letter)
                number_file_name=getLetterStl(number)
                if(letter_file_name != None):
                    letter_mesh=vedo.Mesh(letter_file_name)
                    letter_mesh.pos(index*6,0,0)
                    
                    meshes.append(letter_mesh)
                    index+=1
        for letter in number:
            if letter==' ':
                index+=1
            else:    
                number_file_name=getLetterStl(letter)
                if(number_file_name != None):
                    number_mesh=vedo.Mesh(number_file_name)
                    number_mesh.pos(index*3,-8,0)
                    
                    meshes.append(number_mesh)
                    index+=1

        mesh=vedo.merge(meshes)
        mesh.rotateX(rotateX)
        mesh.rotateY(rotateY)
        mesh.rotateZ(rotateZ)
        vedo.show(mesh, axes=True)
        vedo.write(mesh, outputStl)
        return mesh
    except Exception as e:
        print("**************\nOops! Exception occurred.")
        print(e)
        print("**************")
        return None


# def getMergeStl(teethFiles,attachmentFiles,gumFile,alignerNumberStlToEmboss,outputFile) -> str:
def getMergeStl(teethFiles,attachmentFiles,gumFile,outputFile) -> str:
    try:
        meshes=[]
        index = 0
        print(teethFiles)
        for file in os.listdir(teethFiles):
            print(file)
            teeth = vedo.Mesh(teethFiles+ '/' + file)
            meshes.append(teeth)
        print("teeth steps done")    
        for file in os.listdir(attachmentFiles):
            print(attachmentFiles+ '/' + file)
            attachment = vedo.Mesh(attachmentFiles+ '/' + file)
            meshes.append(attachment)
        print("attachment done")
        gum = vedo.Mesh(gumFile)
        meshes.append(gum)  
        print("gum file done")
        # alignerNumber = vedo.Mesh(alignerNumberStlToEmboss)
        # meshes.append(alignerNumber)
        # print("alligner done")
        # vedo.show(meshes, axes=True)
        mesh=vedo.merge(meshes)
        vedo.write(mesh, outputFile)
        return 
       
        
    except Exception as e:
        print("**************\nOops! Exception occurred.")
        print(e)
        print("**************")
        return None

def getMergeLogo(nameStl,logoStl,outputFile) -> str:
    try:
        meshes=[]
        index = 0
       
        
        nameMesh = vedo.Mesh(nameStl)
        meshes.append(nameMesh)

        logoMesh = vedo.Mesh(logoStl)
        meshes.append(logoMesh)
        
        # vedo.show(meshes, axes=True)
        mesh=vedo.merge(meshes)
        vedo.write(mesh, outputFile)
        return 
       
        
    except Exception as e:
        print("**************\nOops! Exception occurred.")
        print(e)
        print("**************")
        return None

# getStlWithRotationAndTranslation("Neural Hive","7","testoutput.stl", 90, 0, 0,-10, -10, -10)
# getStlWithTranslation("Neural Hive","7","testoutput.stl", -10, -10, -10)
# getStlWithRotation("Neural Hive","7","testoutput.stl", 90, 0, 0)
# getStl("Neural Hive","7","testoutput.stl")
# getMergeStl('op.stl')