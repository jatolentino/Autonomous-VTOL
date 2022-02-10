function [Hdb,FdbT,Cdb,Adci,C_cm_g,y_max_global,y_min_global] = attitudeControl(Ad,Bd,Cd,Dd,hz,y_max,y_min)
    pCs = plantConstants1;
    Q = pCs.Q;
    S = pCs.S;
    R = pCs.R;
    Ccm = pCs.Ccm; % [0 0 0 0 0 0 1 0 0;0 0 0 0 0 0 0 1 0;0 0 0 0 0 0 0 0 1]
    y_max_global=zeros(length(y_max(:,1))*hz,length(y_max(1,:)));
    y_min_global=zeros(length(y_min(:,1))*hz,length(y_min(1,:)));
    
    
    % Ad6x6  Bd6x3  Cd3x6  Dd3x3
    Aaug_tilde = [Ad Bd;zeros(flip(size(Bd))) eye(size(Bd,2))]; % size(Bd,2) is column #s of Bd  Augtilde:9x9
    Baug_tilde = [Bd;eye(size(Bd,2))];                          % Baugtilde: 9x3
    Caug_tilde = [Cd zeros(size(Cd,1))];                        % size(Cd,1) is raw #s of Cd   Caugtilde: 3x9
    Daug_tilde = Dd;                                            % Daugtilde: 3x3
    
    CtQC = Caug_tilde'*Q*Caug_tilde;
    CtSC = Caug_tilde'*S*Caug_tilde;
    
    QC = Q*Caug_tilde;
    SC = S*Caug_tilde;
    
    Qdb = zeros(hz*size(CtQC));
    Tdb = zeros(hz*size(QC));
    Rdb = zeros(hz*size(R));
    Cdb = zeros(size(Baug_tilde,1)*hz,size(Baug_tilde,2)*hz);
    Adci = zeros(size(Aaug_tilde,1)*hz,size(Aaug_tilde,2));
    C_cm_gx = zeros(length(Baug_tilde(:,1))*hz,length(Baug_tilde(1,:))*hz);
    C_cm_g = C_cm_gx';
    
    for i = 1:hz
       if i == hz
           Qdb(1+length(CtSC(:,1))*(i-1):length(CtSC(:,1))*i,1+length(CtSC(1,:))*(i-1):length(CtSC(1,:))*i) = CtSC;
           Tdb(1+length(SC(:,1))*(i-1):length(SC(:,1))*i,1+length(SC(1,:))*(i-1):length(SC(1,:))*i) = SC;
           C_cm_g(1+length(Ccm(:,1))*(i-1):length(Ccm(:,1))*i,1+length(Ccm(1,:))*(i-1):length(Ccm(1,:))*i)=Ccm;
       else
           Qdb(1+length(CtQC(:,1))*(i-1):length(CtQC(:,1))*i,1+length(CtQC(1,:))*(i-1):length(CtQC(1,:))*i) = CtQC;
           Tdb(1+length(QC(:,1))*(i-1):length(QC(:,1))*i,1+length(QC(1,:))*(i-1):length(QC(1,:))*i) = QC;
           C_cm_g(1+length(Ccm(:,1))*(i-1):length(Ccm(:,1))*i,1+length(Ccm(1,:))*(i-1):length(Ccm(1,:))*i)=Ccm;
       end
       Rdb(1+length(R(:,1))*(i-1):length(R(:,1))*i,1+length(R(1,:))*(i-1):length(R(1,:))*i) = R;
       
       for j = 1:hz
           if j<=i
               Cdb(1+length(Baug_tilde(:,1))*(i-1):length(Baug_tilde(:,1))*i,...
                   1+length(Baug_tilde(1,:))*(j-1):length(Baug_tilde(1,:))*j) = Aaug_tilde^(i-j)*Baug_tilde;
           end
       end
       Adci(1+length(Aaug_tilde(:,1))*(i-1):length(Aaug_tilde(:,1))*i,1:length(Aaug_tilde(1,:))) = Aaug_tilde^(i);
       y_max_global(length(y_max(:,1))*i-2:length(y_max(:,1))*(i),1) = y_max(:,1);
       y_min_global(length(y_min(:,1))*i-2:length(y_min(:,1))*(i),1) = y_min(:,1);
    end
    Hdb = Cdb'*Qdb*Cdb + Rdb;
    FdbT = [Adci'*Qdb*Cdb ; - Tdb*Cdb];
end
    
